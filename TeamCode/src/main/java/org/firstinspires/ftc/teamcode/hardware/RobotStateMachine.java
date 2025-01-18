package org.firstinspires.ftc.teamcode.hardware;
import static org.firstinspires.ftc.teamcode.autonomous.Chamber.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import static org.firstinspires.ftc.teamcode.hardware.Arm.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.robotStates.CHAMBER;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.robotStates.WALL;
import static java.lang.Math.*;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.ParCommand;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.command.StateMachine;
import org.firstinspires.ftc.teamcode.command.StateMachineBuilder;
import org.firstinspires.ftc.teamcode.command.WaitCommand;
import org.firstinspires.ftc.teamcode.control.AsymProfile.*;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.TrajCommandBuilder;
import org.firstinspires.ftc.teamcode.movement.Vec;

public class RobotStateMachine {
    public enum robotStates {
        EXTEND, EXTEND_GRAB, GRABBED, BUCKET, WALL, CHAMBER, CLIMB, CLIMBED
    }
    public static StateMachine<robotStates> get(CommandOpMode opMode, Robot robot, robotStates state) {
        StateMachineBuilder<robotStates> builder = new StateMachineBuilder<robotStates>(opMode)
                .addState(robotStates.values())
                .addTransition(robotStates.EXTEND, robotStates.EXTEND_GRAB, new SeqCommand(
                        new WaitCommand(t -> robot.arm.setArm(armRest(robot.arm.armPos().wristRot)),
                                0.15, t -> robot.arm.setClaw(true), robot.arm),
                        new WaitCommand(0.15, t -> robot.arm.setArm(armGrab(robot.arm.armPos().wristRot)))))
                .addTransition(robotStates.EXTEND_GRAB, robotStates.EXTEND,
                        new WaitCommand(t -> robot.arm.setClaw(false), 0.15, robot.arm))
                .addTransition(robotStates.EXTEND, robotStates.GRABBED, new SeqCommand(
                        new SeqCommand(
                                new WaitCommand(t -> robot.arm.setArm(new ArmPosition(0, 0, robot.arm.armPos().wristRot)),
                                        0.15, t -> robot.arm.setClaw(true), robot.arm),
                                new WaitCommand(0.15, t -> robot.arm.setArm(armGrabbed), robot.arm),
                                robot.lift.goBack(false, true))))
                .addTransition(robotStates.EXTEND_GRAB, robotStates.GRABBED, new ParCommand(
                        new WaitCommand(t -> robot.arm.setArm(armGrabbed), 0.15, robot.arm),
                        robot.lift.goBack(false, true)))
                .addTransition(robotStates.GRABBED, robotStates.EXTEND, a -> new ParCommand(
                        robot.lift.goTo(LiftPosition.inverse(((Pose)a[0]).vec())),
                        new SeqCommand(
                            FnCommand.once(t -> robot.arm.setArm(armGrab(-PI/2))),
                            robot.arm.setGrab(((Pose)a[0]).h, robot.lift),
                            new FnCommand(t -> {}, t -> {}, (t, b) -> robot.arm.setClaw(false),
                                    t -> t > robot.lift.restTime() - 0.15))))
                .addTransition(robotStates.GRABBED, robotStates.BUCKET, a -> robot.lift.goTo((LiftPosition)a[0]))
                .addTransition(robotStates.BUCKET, robotStates.BUCKET, a -> robot.lift.goTo((LiftPosition)a[0]))
                .addTransition(robotStates.BUCKET, robotStates.EXTEND, a -> new SeqCommand(
                        new WaitCommand(t -> robot.arm.setArm(armBucket), 0.1, robot.arm),
                        new WaitCommand(t -> robot.arm.setClaw(false), 0.15, t -> robot.arm.setArm(armGrab(-PI/2))),
                        robot.lift.goBack(true, false),
                        new ParCommand(
                                robot.lift.goTo(LiftPosition.inverse(((Pose)a[0]).vec())),
                                robot.arm.setGrab(((Pose)a[0]).h, robot.lift))))
                .addTransition(robotStates.GRABBED, robotStates.WALL, a -> new SeqCommand(
                        new ParCommand(
                            robot.lift.goTo(liftWall1),
                            new FnCommand(t -> robot.arm.setArm(armWall1), t -> {}, (t, b) -> {
                                robot.arm.setArm(armWall2);
                                robot.arm.setClaw(false);}, t -> t > robot.lift.restTime() - 0.15, robot.arm))))
                .addTransition(robotStates.WALL, robotStates.CHAMBER, a -> new SeqCommand(
                        new WaitCommand(t -> robot.arm.setArm(armWall3), 0.15, robot.arm),
                        new WaitCommand(t -> robot.arm.setClaw(true), 0.15, robot.arm),
                        robot.lift.goTo(liftWall2),
                        new ParCommand(
                                robot.lift.wallTo(liftChamber1),
                                new WaitCommand(t -> robot.arm.setArm(armWall1), 0.25,
                                        t -> robot.arm.setArm(armChamber), robot.arm))))
                .addTransition(robotStates.WALL, robotStates.GRABBED, a -> new SeqCommand(
                        new WaitCommand(t -> robot.arm.setArm(armWall3), 0.15, robot.arm),
                        new WaitCommand(t -> robot.arm.setClaw(true), 0.15, robot.arm),
                        robot.lift.goTo(liftWall2),
                        new ParCommand(
                                robot.lift.goBack(false, true),
                                new WaitCommand(t -> robot.arm.setArm(armWall1), 0.25,
                                        t -> robot.arm.setArm(armGrabbed), robot.arm))))
                .addTransition(robotStates.CHAMBER, robotStates.WALL, a -> new SeqCommand(
                        robot.lift.specimen(),
                        new ParCommand(
                                robot.lift.toWall(),
                                new SeqCommand(
                                        new WaitCommand(t -> robot.arm.setClaw(false), 0.15),
                                        new FnCommand(t -> robot.arm.setArm(armWall1), t -> {}, (t, b) -> {
                                        robot.arm.setArm(armWall2);
                                        robot.arm.setClaw(false);}, t -> t > robot.lift.restTime() - 0.15, robot.arm)))))
                .addTransition(robotStates.GRABBED, robotStates.CHAMBER, a -> new ParCommand(
                        robot.lift.goTo(liftChamber1),
                        FnCommand.once(t -> robot.arm.setArm(armChamber), robot.arm)))
                .addTransition(robotStates.CHAMBER, robotStates.EXTEND, a -> new SeqCommand(
                        robot.lift.specimen(),
                        new ParCommand(
                            robot.lift.goBack(false, false),
                            new WaitCommand(t -> robot.arm.setClaw(false), 0.15,
                                    t -> robot.arm.setArm(armGrab(-PI)), robot.arm)),
                        new ParCommand(
                                robot.lift.goTo(LiftPosition.inverse(((Pose)a[0]).vec())),
                                robot.arm.setGrab(((Pose)a[0]).h, robot.lift))))
                .addTransition(robotStates.WALL, robotStates.WALL, a -> new SeqCommand(
                        new WaitCommand(t -> {
                            robot.drive.setTrajectory(null);
                            robot.drive.setPowers(new Vec(-0.5, 0), 0);}, (double)a[0]),
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
                .addTransition(robotStates.GRABBED, robotStates.CLIMB, new ParCommand(
                        FnCommand.once(t -> {
                            robot.arm.setArm(armRest(-PI/2));
                            robot.arm.setClaw(false);}, robot.arm),
                        robot.lift.goTo(climb1)))
                .addTransition(robotStates.CLIMB, robotStates.GRABBED, new ParCommand(
                        FnCommand.once(t -> {
                            robot.arm.setArm(armGrabbed);
                            robot.arm.setClaw(true);}),
                        robot.lift.goBack(false, true)))
                .addTransition(robotStates.CLIMB, robotStates.CLIMBED, robot.lift.climb());
        return builder.build(state);
    }
}