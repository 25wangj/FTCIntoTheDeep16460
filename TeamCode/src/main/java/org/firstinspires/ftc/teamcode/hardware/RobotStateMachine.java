package org.firstinspires.ftc.teamcode.hardware;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import static org.firstinspires.ftc.teamcode.hardware.Arm.*;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.ParCommand;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.command.StateMachine;
import org.firstinspires.ftc.teamcode.command.StateMachineBuilder;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.command.WaitCommand;

public class RobotStateMachine {
    public enum robotStates {
        INTAKE, EXTEND, EXTEND_GRAB, GRABBED, SPECIMEN, BUCKET, SIDE_CHAMBER, BACK_CHAMBER, CLIMB, CLIMBED
    }
    public static StateMachine<robotStates> get(CommandOpMode opMode, Robot robot, robotStates state) {
        Subsystem[] subsystems = {robot.lift, robot.intake, robot.arm};
        StateMachineBuilder<robotStates> builder = new StateMachineBuilder<robotStates>(opMode)
                .addState(robotStates.values())
                .addTransition(robotStates.INTAKE, robotStates.EXTEND, a -> new ParCommand(
                                robot.lift.goTo((LiftPosition)a[0]),
                                FnCommand.once(t -> {
                                    robot.arm.setGrab((Double)a[1], robot.lift);
                                    robot.intake.set(0);}, robot.arm)))
                .addTransition(robotStates.INTAKE, robotStates.GRABBED, new SeqCommand(
                        new WaitCommand(t -> {
                            robot.arm.setArm(armRest);
                            robot.intake.set(0);}, 0.15,
                                t -> robot.arm.setClaw(true), subsystems),
                        new WaitCommand(0.25, t -> {
                            robot.arm.setArm(armGrabbed);
                            robot.intake.set(-0.25);}, subsystems),
                        new WaitCommand(0.15, t -> robot.intake.set(0), subsystems)))
                .addTransition(robotStates.EXTEND, robotStates.EXTEND_GRAB, new WaitCommand(t ->
                        robot.arm.setArm(new ArmPosition(0, 0, robot.arm.armPos().wristRot)), 0.15, robot.arm))
                .addTransition(robotStates.EXTEND_GRAB, robotStates.EXTEND, new WaitCommand(t ->
                        robot.arm.setArm(new ArmPosition(armUp, 0, robot.arm.armPos().wristRot)), 0.15, robot.arm))
                .addTransition(robotStates.EXTEND_GRAB, robotStates.GRABBED, new SeqCommand(
                        new WaitCommand(t -> robot.arm.setClaw(true), 0.25, robot.arm),
                        new WaitCommand(t -> robot.arm.setArm(armGrabbed), 0.15, robot.arm),
                        robot.lift.goBack(false)))
                .addTransition(robotStates.GRABBED, robotStates.INTAKE, new WaitCommand(t -> {
                    robot.arm.setArm(armGrab);
                    robot.arm.setClaw(false);
                    robot.intake.set(0.3);}, 0.25, subsystems))
                .addTransition(robotStates.GRABBED, robotStates.SPECIMEN, new ParCommand(
                        FnCommand.once(t -> {
                            robot.intake.set(0.3);
                        }, robot.intake),
                        robot.lift.goTo(liftSpecimen)))
                .addTransition(robotStates.SPECIMEN, robotStates.INTAKE, new SeqCommand(
                        new WaitCommand(t -> robot.arm.setClaw(false), 0.25,
                                t -> robot.arm.setArm(armGrab), robot.arm),
                        robot.lift.goBack(false)))
                .addTransition(robotStates.GRABBED, robotStates.BUCKET, a -> new ParCommand(
                        FnCommand.once(t -> robot.arm.setArm(armBucket)),
                        robot.lift.goTo((LiftPosition)a[0])))
                .addTransition(robotStates.BUCKET, robotStates.BUCKET, a -> robot.lift.goTo((LiftPosition)a[0]))
                .addTransition(robotStates.BUCKET, robotStates.INTAKE, new SeqCommand(
                        new WaitCommand(t -> {
                            robot.arm.setClaw(false);
                            robot.arm.setArm(armGrabbed);
                        }, 0.25, subsystems),
                        new WaitCommand(t -> {
                            robot.arm.setArm(armGrab);
                            robot.intake.set(0.3);}, 0.15, subsystems),
                        robot.lift.goBack(true)))
                .addTransition(robotStates.GRABBED, robotStates.SIDE_CHAMBER, a -> new SeqCommand(
                        new ParCommand(
                            new WaitCommand(0.25, t -> robot.arm.setArm(armSideChamber1), robot.arm),
                            robot.lift.goTo((LiftPosition)a[0])),
                        FnCommand.once(t -> robot.arm.setArm(armSideChamber2))))
                .addTransition(robotStates.SIDE_CHAMBER, robotStates.SIDE_CHAMBER, a -> robot.lift.goTo((LiftPosition)a[0]))
                .addTransition(robotStates.SIDE_CHAMBER, robotStates.INTAKE, new SeqCommand(
                        robot.lift.specimen(),
                        new WaitCommand(t -> robot.arm.setClaw(false), 0.15, t -> {
                            robot.intake.set(0.3);
                            robot.arm.setArm(armSideChamber1);}, subsystems),
                        new ParCommand(
                            new WaitCommand(0.25, t -> robot.arm.setArm(armGrab), robot.arm),
                            robot.lift.goBack(false))))
                .addTransition(robotStates.GRABBED, robotStates.BACK_CHAMBER, a -> new ParCommand(
                        FnCommand.once(t -> robot.arm.setArm(armBackChamber), robot.arm),
                        robot.lift.goTo((LiftPosition)a[0])))
                .addTransition(robotStates.BACK_CHAMBER, robotStates.BACK_CHAMBER, a -> robot.lift.goTo((LiftPosition)a[0]))
                .addTransition(robotStates.BACK_CHAMBER, robotStates.INTAKE, new SeqCommand(
                        robot.lift.specimen(),
                        new WaitCommand(t -> robot.arm.setClaw(false), 0.15, t -> {
                            robot.arm.setArm(armGrab);
                            robot.intake.set(0.3);}, subsystems),
                        robot.lift.goBack(false)))
                .addTransition(robotStates.INTAKE, robotStates.CLIMB, new ParCommand(
                        FnCommand.once(t -> {
                            robot.arm.setArm(armRest);
                            robot.intake.set(0);}),
                        robot.lift.goTo(climb1)))
                .addTransition(robotStates.CLIMB, robotStates.INTAKE, new ParCommand(
                        FnCommand.once(t -> {
                            robot.arm.setArm(armGrab);
                            robot.intake.set(0.3);}),
                        robot.lift.goBack(false)))
                .addTransition(robotStates.CLIMB, robotStates.CLIMBED, robot.lift.climb());
        return builder.build(state);
    }
}