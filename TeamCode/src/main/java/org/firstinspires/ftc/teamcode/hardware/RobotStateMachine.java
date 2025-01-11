package org.firstinspires.ftc.teamcode.hardware;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import static org.firstinspires.ftc.teamcode.hardware.Arm.*;
import static java.lang.Math.*;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.ParCommand;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.command.StateMachine;
import org.firstinspires.ftc.teamcode.command.StateMachineBuilder;
import org.firstinspires.ftc.teamcode.command.WaitCommand;
import org.firstinspires.ftc.teamcode.movement.Pose;
public class RobotStateMachine {
    public enum robotStates {
        EXTEND, EXTEND_GRAB, GRABBED, BUCKET, SIDE_CHAMBER, BACK_CHAMBER, CLIMB, CLIMBED
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
                        new SeqCommand(
                            new WaitCommand(t -> {
                                robot.arm.setArm(armGrab(-PI/2));
                                robot.arm.setClaw(false);}, 0.15, robot.arm),
                            robot.arm.setGrab(((Pose)a[0]).h, robot.lift)),
                        robot.lift.goTo(LiftPosition.inverse(((Pose)a[0]).vec()))))
                .addTransition(robotStates.GRABBED, robotStates.BUCKET, a -> robot.lift.goTo((LiftPosition)a[0]))
                .addTransition(robotStates.BUCKET, robotStates.BUCKET, a -> robot.lift.goTo((LiftPosition)a[0]))
                .addTransition(robotStates.BUCKET, robotStates.EXTEND, a -> new SeqCommand(
                        new WaitCommand(t -> robot.arm.setArm(armBucket), 0.1, robot.arm),
                        new WaitCommand(t -> robot.arm.setClaw(false), 0.15, t -> robot.arm.setArm(armGrab(-PI/2))),
                        robot.lift.goBack(true, false),
                        new ParCommand(
                                robot.lift.goTo(LiftPosition.inverse(((Pose)a[0]).vec())),
                                robot.arm.setGrab(((Pose)a[0]).h, robot.lift))))
                .addTransition(robotStates.GRABBED, robotStates.SIDE_CHAMBER, a -> new SeqCommand(
                        new ParCommand(
                            robot.lift.goTo((LiftPosition)a[0]),
                            new SeqCommand(
                                    new WaitCommand(0.25, t -> robot.arm.setArm(armSideChamber1), robot.arm),
                                    new FnCommand(t -> {}, t -> {}, (t, b) -> robot.arm.setArm(armSideChamber2),
                                            t -> t > robot.lift.restTime() - 0.15)))))
                .addTransition(robotStates.SIDE_CHAMBER, robotStates.EXTEND, a -> new SeqCommand(
                        robot.lift.specimen(),
                        new WaitCommand(t -> robot.arm.setClaw(false), 0.15,
                                t -> robot.arm.setArm(armSideChamber1), robot.arm),
                        new ParCommand(
                                robot.lift.goBack(false, false),
                                new WaitCommand(0.25, t -> robot.arm.setArm(armGrab(-1.83)), robot.arm)),
                        new ParCommand(
                                robot.lift.goTo(LiftPosition.inverse(((Pose)a[0]).vec())),
                                robot.arm.setGrab(((Pose)a[0]).h, robot.lift))))
                .addTransition(robotStates.GRABBED, robotStates.BACK_CHAMBER, a -> new ParCommand(
                        FnCommand.once(t -> robot.arm.setArm(armBackChamber), robot.arm),
                        robot.lift.goTo((LiftPosition)a[0])))
                .addTransition(robotStates.BACK_CHAMBER, robotStates.EXTEND, a -> new SeqCommand(
                        robot.lift.specimen(),
                        new WaitCommand(t -> robot.arm.setClaw(false), 0.15,
                                t -> robot.arm.setArm(armGrab(0)), robot.arm),
                        new SeqCommand(
                                robot.lift.goBack(false, false),
                                new ParCommand(
                                        robot.lift.goTo(LiftPosition.inverse(((Pose)a[0]).vec())),
                                        robot.arm.setGrab(((Pose)a[0]).h, robot.lift)))))
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