package org.firstinspires.ftc.teamcode.hardware;
import static org.firstinspires.ftc.teamcode.autonomous.Chamber.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import static org.firstinspires.ftc.teamcode.hardware.Arm.*;
import static org.firstinspires.ftc.teamcode.hardware.Vision.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.MovementType.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.RobotStates.*;
import static java.lang.Math.*;

import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.LazyCommand;
import org.firstinspires.ftc.teamcode.command.ParCommand;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.command.StateMachine;
import org.firstinspires.ftc.teamcode.command.StateMachineBuilder;
import org.firstinspires.ftc.teamcode.command.WaitCommand;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.TrajCommandBuilder;
import org.firstinspires.ftc.teamcode.movement.Vec;
import org.firstinspires.ftc.teamcode.vision.VisionValueStorage;
import org.opencv.core.Point3;

import java.util.ArrayList;
import java.util.List;

public class RobotStateMachine {
    public enum RobotStates {
        EXTEND, EXTEND_GRAB, GRABBED, BUCKET, WALL, CHAMBER, CLIMB, CLIMBED, CAMERA
    }
    public static StateMachine<RobotStates> get(CommandOpMode opMode, Robot robot, RobotStates state) {
        StateMachineBuilder<RobotStates> builder = new StateMachineBuilder<RobotStates>(opMode)
                .addState(RobotStates.values())
                .addTransition(EXTEND, EXTEND_GRAB, new SeqCommand(
                        new WaitCommand(t -> robot.arm.setArm(armRest(robot.arm.armPos().wristRot)),
                                0.1, t -> robot.arm.setClaw(true), robot.arm),
                        new WaitCommand(0.15, t -> robot.arm.setArm(armHalf(robot.arm.armPos().wristRot)))))
                .addTransition(EXTEND_GRAB, EXTEND,
                        new WaitCommand(t -> {
                            robot.arm.setClaw(false);
                            robot.arm.setArm(armGrab(robot.arm.armPos().wristRot));
                        }, 0.15, robot.arm))
                .addTransition(EXTEND_GRAB, GRABBED, new ParCommand(
                        FnCommand.once(t -> robot.arm.setArm(armGrabbed), robot.arm),
                        robot.lift.goBack()))
                .addTransition(EXTEND, GRABBED, new SeqCommand(
                        new WaitCommand(t -> robot.arm.setArm(armRest(robot.arm.armPos().wristRot)),
                                0.1, t -> robot.arm.setClaw(true), robot.arm),
                        new WaitCommand(0.15, t -> robot.arm.setArm(armHalfGrabbed), robot.arm),
                        robot.lift.turretTo(0),
                        FnCommand.once(t -> robot.arm.setArm(armGrabbed), robot.arm),
                        robot.lift.goBack()))
                .addTransition(GRABBED, EXTEND, a -> new ParCommand(
                        robot.lift.goTo(LiftPosition.inverse(((Pose)a[0]).vec()), PIVOT_FIRST),
                        new FnCommand(t -> robot.arm.setArm(armGrab(-PI/2)),
                                t -> {}, (t, b) -> robot.arm.setClaw(false),
                                t -> t > robot.lift.restTime() - 0.15),
                        robot.arm.setGrab(((Pose)a[0]).h, robot.lift)))
                .addTransition(EXTEND, BUCKET, a -> new SeqCommand(
                        new WaitCommand(t -> robot.arm.setArm(armRest(robot.arm.armPos().wristRot)),
                                0.1, t -> robot.arm.setClaw(true), robot.arm),
                        new WaitCommand(0.15, t -> robot.arm.setArm(armHalfGrabbed), robot.arm),
                        robot.lift.turretTo(0),
                        new ParCommand(
                            robot.lift.goTo((LiftPosition)a[0], PIVOT_RETRACT),
                            new FnCommand(t -> robot.arm.setArm(armGrabbed), t -> {},
                                    (t, b) -> robot.arm.setArm(armBucket1), t -> robot.lift.liftPos(t).pivotAng != 0))))
                .addTransition(GRABBED, BUCKET, a -> new ParCommand(
                        FnCommand.once(t -> robot.arm.setArm(armBucket1)),
                        robot.lift.goTo((LiftPosition)a[0], PIVOT_FIRST)))
                .addTransition(BUCKET, BUCKET, a -> robot.lift.goTo((LiftPosition)a[0], PIVOT_FIRST))
                .addTransition(BUCKET, EXTEND, a -> new SeqCommand(
                        new WaitCommand(t -> robot.arm.setArm(armBucket2), 0.1, robot.arm),
                        new WaitCommand(t -> robot.arm.setClaw(false), 0.15, t -> robot.arm.setArm(armGrab(-PI/2))),
                        robot.lift.pivotTo(PI/2),
                        new ParCommand(
                                robot.lift.goTo(LiftPosition.inverse(((Pose)a[0]).vec()), LIFT_RETRACT),
                                robot.arm.setGrab(((Pose)a[0]).h, robot.lift))))
                .addTransition(BUCKET, GRABBED, new SeqCommand(
                        new WaitCommand(t -> robot.arm.setArm(armBucket2), 0.1, robot.arm),
                        new WaitCommand(t -> robot.arm.setClaw(false), 0.15, t -> robot.arm.setArm(armGrabbed)),
                        robot.lift.pivotTo(PI/2),
                        new ParCommand(
                            robot.lift.goTo(LiftPosition.inverse(new Vec(0, 0)), LIFT_RETRACT),
                            FnCommand.once(t -> robot.arm.setClaw(true)))))
                .addTransition(BUCKET, CAMERA, new SeqCommand(
                        new WaitCommand(t -> robot.arm.setArm(armBucket2), 0.1, robot.arm),
                        new WaitCommand(t -> robot.arm.setClaw(false), 0.15, t -> robot.arm.setArm(armGrab(-PI/2))),
                        robot.lift.pivotTo(PI/2),
                        new ParCommand(
                            robot.lift.goTo(liftCamera, LIFT_RETRACT),
                            FnCommand.once(t -> robot.arm.setArm(armGrab(0))))))
                .addTransition(GRABBED, WALL, a -> new SeqCommand(
                        new ParCommand(
                            robot.lift.goTo(liftWall1, PIVOT_FIRST),
                            new FnCommand(t -> robot.arm.setArm(armWall1), t -> {}, (t, b) -> {
                                robot.arm.setArm(armWall2);
                                robot.arm.setClaw(false);}, t -> t > robot.lift.restTime(), robot.arm))))
                .addTransition(WALL, CHAMBER, a -> new SeqCommand(
                        new WaitCommand(t -> robot.arm.setArm(armWall3), 0.1, robot.arm),
                        new WaitCommand(t -> robot.arm.setClaw(true), 0.15, robot.arm),
                        robot.lift.goTo(liftWall2, PIVOT_FIRST),
                        new ParCommand(
                                robot.lift.goTo(liftChamber, TURRET_FIRST),
                                new WaitCommand(t -> robot.arm.setArm(armWall1), 0.25,
                                        t -> robot.arm.setArm(armChamber), robot.arm))))
                .addTransition(WALL, GRABBED, a -> new SeqCommand(
                        new WaitCommand(t -> robot.arm.setArm(armWall3), 0.1, robot.arm),
                        new WaitCommand(t -> robot.arm.setClaw(true), 0.15, robot.arm),
                        robot.lift.goTo(liftWall2, PIVOT_FIRST),
                        new ParCommand(
                                robot.lift.goBack(),
                                new WaitCommand(t -> robot.arm.setArm(armWall1), 0.25,
                                        t -> robot.arm.setArm(armGrabbed), robot.arm))))
                .addTransition(CHAMBER, WALL, a -> new SeqCommand(
                        robot.lift.specimen(),
                        new WaitCommand(t -> robot.arm.setClaw(false), 0.1),
                        new ParCommand(
                                robot.lift.goTo(liftWall1, TURRET_LAST),
                                new SeqCommand(
                                        new FnCommand(t -> robot.arm.setArm(armWall1), t -> {}, (t, b) -> {
                                        robot.arm.setArm(armWall2);
                                        robot.arm.setClaw(false);}, t -> t > robot.lift.restTime() - 0.15, robot.arm)))))
                .addTransition(GRABBED, CHAMBER, a -> new ParCommand(
                        robot.lift.goTo(liftChamber, PIVOT_FIRST),
                        FnCommand.once(t -> robot.arm.setArm(armChamber), robot.arm)))
                .addTransition(CHAMBER, EXTEND, a -> new SeqCommand(
                        robot.lift.specimen(),
                        new WaitCommand(t -> robot.arm.setClaw(false), 0.1,
                                t -> robot.arm.setArm(armGrab(-PI)), robot.arm),
                        new ParCommand(
                            robot.lift.goTo(LiftPosition.inverse(((Pose)a[0]).vec()), LIFT_RETRACT),
                            robot.arm.setGrab(((Pose)a[0]).h, robot.lift))))
                .addTransition(WALL, WALL, a -> new SeqCommand(
                        new WaitCommand(t -> {
                            robot.drive.setTrajectory(null);
                            robot.drive.setPowers(new Vec(-0.3, 0), 0);}, (double)a[0]),
                        new WaitCommand(t -> robot.stateMachine.transition(WALL, CHAMBER), 0.3),
                        new TrajCommandBuilder(robot.drive, wall)
                            .marker(t -> robot.drive.setPose(wall))
                            .marker(1, 0, t -> robot.drive.setTrajectory(null))
                            .marker(robot.stateMachine.getTransition(WALL, CHAMBER))
                            .setMoveConstraints(specConstraints)
                            .lineTo(specimen2.vec().combo(1, new Vec(0.5, 0), 1))
                            .marker(1, -0.15, robot.stateMachine.getTransition(CHAMBER, WALL))
                            .lineTo(wall)
                            .build(opMode.scheduler)))
                .addTransition(WALL, EXTEND, a -> new SeqCommand(
                        new WaitCommand(t -> {
                            robot.stateMachine.transition(WALL, CHAMBER);
                            robot.drive.setTrajectory(null);
                            robot.drive.setPowers(new Vec(-0.5, 0), 0);}, 0.3),
                        new TrajCommandBuilder(robot.drive, wall)
                                .marker(t -> robot.drive.setPose(wall))
                                .marker(1, 0, t -> robot.drive.setTrajectory(null))
                                .marker(robot.stateMachine.getTransition(WALL, CHAMBER))
                                .setMoveConstraints(specConstraints)
                                .lineTo(specimen2.vec().combo(1, new Vec(0, -0.5), 1))
                                .marker(1, -0.15, robot.stateMachine.getTransition(CHAMBER, EXTEND, a))
                                .build(opMode.scheduler)))
                .addTransition(GRABBED, CAMERA, new ParCommand(
                        FnCommand.once(t -> {
                            robot.arm.setArm(armGrab(0));
                            robot.arm.setClaw(false);}, robot.arm),
                        robot.lift.goTo(liftCamera, PIVOT_FIRST)))
                .addTransition(CAMERA, EXTEND, a -> new SeqCommand(
                        new FnCommand(t -> {
                            VisionValueStorage.takeFrame = true;
                            VisionValueStorage.yellowPoses = new ArrayList<>();
                            VisionValueStorage.colorPoses = new ArrayList<>();
                            Vision.sampleTime = t;
                        }, t -> {}, (t, b) -> {
                            List<Point3> poses = new ArrayList<>();
                            switch ((Priority)a[0]) {
                                case YELLOW_ONLY:
                                    poses.addAll(VisionValueStorage.yellowPoses);
                                    break;
                                case COLOR_ONLY:
                                    poses.addAll(VisionValueStorage.colorPoses);
                                    break;
                                case YELLOW_COLOR:
                                    poses.addAll(VisionValueStorage.yellowPoses);
                                    poses.addAll(VisionValueStorage.colorPoses);
                                    break;
                                case COLOR_YELLOW:
                                    poses.addAll(VisionValueStorage.colorPoses);
                                    poses.addAll(VisionValueStorage.yellowPoses);
                                    break;
                            }
                            samplePose = new Pose(12, 0, 0);
                            for (Point3 p : poses) {
                                if (p.x > liftXMin && p.x < liftXMax && abs(p.y) < liftYMax) {
                                    samplePose = new Pose(p);
                                    break;
                                }
                            }
                            System.out.println(t - Vision.sampleTime);
                        }, t -> !VisionValueStorage.takeFrame || (t - Vision.sampleTime > maxDelay)),
                        new LazyCommand(() -> new ParCommand(
                                robot.lift.goTo(LiftPosition.inverse(samplePose.vec()), TURRET_FIRST, pivotCameraConstraints),
                                FnCommand.once(t -> robot.arm.setArm(armHalf(0))),
                                robot.arm.setGrab(samplePose.h, robot.lift)))
                    )
                )
                .addTransition(GRABBED, CLIMB, new ParCommand(
                        FnCommand.once(t -> {
                            robot.arm.setArm(armRest(-PI/2));
                            robot.arm.setClaw(false);}, robot.arm),
                        robot.lift.goTo(climb1, PIVOT_FIRST)))
                .addTransition(CLIMB, GRABBED, new ParCommand(
                        FnCommand.once(t -> {
                            robot.arm.setArm(armGrabbed);
                            robot.arm.setClaw(true);}),
                        robot.lift.goBack()))
                .addTransition(CLIMB, CLIMBED, robot.lift.climb());
        return builder.build(state);
    }
}