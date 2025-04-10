package org.firstinspires.ftc.teamcode.hardware;
import static org.firstinspires.ftc.teamcode.autonomous.Chamber.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import static org.firstinspires.ftc.teamcode.hardware.Arm.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.MovementType.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.RobotStates.*;
import static java.lang.Math.*;

import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.ParCommand;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.command.StateMachine;
import org.firstinspires.ftc.teamcode.command.StateMachineBuilder;
import org.firstinspires.ftc.teamcode.command.WaitCommand;
import org.firstinspires.ftc.teamcode.movement.TrajCommandBuilder;
import org.firstinspires.ftc.teamcode.movement.Vec;

public class RobotStateMachine {
    public enum RobotStates {
        EXTEND, EXTEND_GRAB, GRABBED, BUCKET, WALL, CHAMBER, CLIMB, CLIMBED
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
                .addTransition(EXTEND, WALL, new SeqCommand(
                        new WaitCommand(t -> robot.arm.setArm(armRest(robot.arm.armPos().wristRot)),
                                0.1, t -> robot.arm.setClaw(true), robot.arm),
                        new WaitCommand(0.15, t -> robot.arm.setArm(armHalfGrabbed), robot.arm),
                        robot.lift.turretTo(0),
                        new ParCommand(
                            robot.lift.goTo(liftWall1, LIFT_RETRACT),
                            new FnCommand(t -> robot.arm.setArm(armWall1), t -> {}, (t, b) -> {
                                robot.arm.setArm(armWall2);
                                robot.arm.setClaw(false);}, t -> t > robot.lift.restTime() - 0.25, robot.arm))))
                .addTransition(GRABBED, EXTEND, a -> new ParCommand(
                        robot.lift.goTo((LiftPosition)a[0], PIVOT_FIRST),
                        new FnCommand(t -> robot.arm.setArm(armGrab(-PI/2)),
                                t -> {}, (t, b) -> robot.arm.setClaw(false),
                                t -> t > robot.lift.restTime() - 0.15),
                        robot.arm.setGrab((double)a[1], robot.lift)))
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
                                robot.lift.goTo((LiftPosition)a[0], LIFT_RETRACT),
                                robot.arm.setGrab((double)a[1], robot.lift))))
                .addTransition(BUCKET, GRABBED, new SeqCommand(
                        new WaitCommand(t -> robot.arm.setArm(armBucket2), 0.1, robot.arm),
                        new WaitCommand(t -> robot.arm.setClaw(false), 0.15, t -> robot.arm.setArm(armGrabbed)),
                        robot.lift.pivotTo(PI/2),
                        new ParCommand(
                            robot.lift.goTo(LiftPosition.inverse(new Vec(0, 0)), LIFT_RETRACT),
                            FnCommand.once(t -> robot.arm.setClaw(true)))))
                .addTransition(GRABBED, WALL, a -> new ParCommand(
                            robot.lift.goTo(liftWall1, PIVOT_FIRST),
                                    new FnCommand(t -> robot.arm.setArm(armWall1), t -> {}, (t, b) -> {
                                        robot.arm.setArm(armWall2);
                                        robot.arm.setClaw(false);}, t -> t > robot.lift.restTime() - 0.25, robot.arm)))
                .addTransition(WALL, CHAMBER, a -> new SeqCommand(
                        new WaitCommand(t -> robot.arm.setArm(armWall3), 0.1, robot.arm),
                        new ParCommand(
                                robot.lift.goTo(liftWall2, PIVOT_FIRST),
                                FnCommand.once(t -> robot.arm.setClaw(true), robot.arm)),
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
                        robot.lift.specimen((boolean)a[0]),
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
                        robot.lift.specimen((boolean)a[3]),
                        new WaitCommand(t -> robot.arm.setClaw(false), 0.1,
                                t -> robot.arm.setArm(armGrab(-PI)), robot.arm),
                        new ParCommand(
                            robot.lift.goTo((LiftPosition)a[0], (MovementType)a[2]),
                            robot.arm.setGrab((double)a[1], robot.lift))))
                .addTransition(WALL, WALL, a -> new ParCommand(
                        new SeqCommand(
                            new WaitCommand(t -> {
                                robot.drive.setTrajectory(null);
                                robot.drive.setPowers(new Vec(-0.5, 0), 0);}, 0.15, t -> robot.drive.setPose(wall1)),
                            new TrajCommandBuilder(robot.drive, wall1)
                                .marker(1, 0, t -> {
                                    robot.drive.setTrajectory(null);
                                    robot.drive.setPowers(new Vec(0, 0), 0);})
                                .setMoveConstraints(specToConstraints)
                                .lineTo(specimen2.vec())
                                .marker(robot.drive.saveTraj())
                                .setMoveConstraints(specBackConstraints)
                                .lineTo(wall2)
                                .pause(0.15)
                                .build(opMode.scheduler)),
                        new SeqCommand(
                            robot.stateMachine.getTransition(WALL, CHAMBER),
                            FnCommand.until(t -> t > robot.drive.curr.tf() - 0.25),
                            robot.stateMachine.getTransition(CHAMBER, WALL, true))))
                .addTransition(WALL, EXTEND, a -> new ParCommand(
                        new SeqCommand(
                                new WaitCommand(t -> {
                                    robot.drive.setTrajectory(null);
                                    robot.drive.setPowers(new Vec(-0.5, 0), 0);}, 0.15, t -> robot.drive.setPose(wall1)),
                                new TrajCommandBuilder(robot.drive, wall1)
                                        .setMoveConstraints(specToConstraints)
                                        .lineTo(specimen2.vec())
                                        .marker(robot.drive.saveTraj())
                                        .build(opMode.scheduler)),
                                new SeqCommand(
                                        robot.stateMachine.getTransition(WALL, CHAMBER),
                                        FnCommand.until(t -> t > robot.drive.curr.tf() - 0.25),
                                        robot.stateMachine.getTransition(CHAMBER, EXTEND, a[0], a[1], a[2], true),
                                        FnCommand.once(t -> {
                                            if ((boolean)a[3]) {
                                                robot.drive.setTrajectory(null);
                                                robot.drive.setPowers(new Vec(0, 0), 0);}}))))
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