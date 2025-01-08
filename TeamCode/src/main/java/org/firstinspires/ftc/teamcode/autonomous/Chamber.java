package org.firstinspires.ftc.teamcode.autonomous;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.robotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import static org.firstinspires.ftc.teamcode.hardware.Arm.*;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.ParCommand;
import org.firstinspires.ftc.teamcode.command.RepeatCommand;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.control.AsymProfile.AsymConstraints;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.TrajCommandBuilder;
@Photon
@Autonomous(name = "Chamber")
public class Chamber extends AbstractAutonomous {
    private AsymConstraints intakeConstraints = new AsymConstraints(10, 30, 30);
    private AsymConstraints pushConstraints = new AsymConstraints(30, 40, 40);
    private AsymConstraints toConstraints = new AsymConstraints(30, 40, 40);
    private AsymConstraints fromConstraints = new AsymConstraints(50, 60, 50);
    private AsymConstraints pushTurnConstraints = new AsymConstraints(4, 8, 4);
    private Pose start = new Pose(-6.5, 63, PI/2);
    private Pose specimen1 = new Pose(-6.5, 31, PI/2);
    private Pose specimen2 = new Pose(-8, 30, 5*PI/6);
    private Pose sample1 = new Pose(-32, 42, 5*PI/4);
    private Pose sample2 = new Pose(-42.5, 42, 5*PI/4);
    private Pose sample3 = new Pose(-53, 42, 5*PI/4);
    private Pose drop1 = new Pose(-31.5, 44, 5*PI/6);
    private Pose drop2 = new Pose(-42, 44, 5*PI/6);
    private Pose preIntake = new Pose(-38, 55, 5*PI/6);
    private Pose intake = new Pose(-44, 59, 5*PI/6);
    private LiftPosition liftPush1 = new LiftPosition(20, PI/4, 0);
    private LiftPosition liftPush2 = new LiftPosition(20, 0, 0);
    private LiftPosition liftPush3 = new LiftPosition(16, PI/4, 0);
    private ArmPosition armPushDown = new ArmPosition(-0.35, 3*PI/4, 1.31);
    private ArmPosition armPushUp = new ArmPosition(0, 3*PI/4, 1.31);
    @Override
    public void initAutonomous() {
        Command traj1 = new TrajCommandBuilder(robot.drive, start)
                .lineTo(specimen1)
                .marker(t -> robot.stateMachine.transition(BACK_CHAMBER, liftHighBackChamber))
                .marker(1, -0.15, t -> robot.stateMachine.transition(EXTEND))
                .setMoveConstraints(pushConstraints)
                .splineTo(sample1, sample1.h)
                .marker(0, 1.1, new ParCommand(
                        FnCommand.once(t -> {
                            robot.arm.setArm(armPushUp);
                            robot.arm.setClaw(true);}),
                        robot.lift.goTo(liftPush1)
                ))
                .pause(0.25)
                .marker(t -> robot.arm.setArm(armPushDown))
                .setTurnConstraints(pushTurnConstraints)
                .lineTo(drop1)
                .marker(robot.lift.goTo(liftPush2))
                .marker(1, -0.15, t -> robot.arm.setArm(armPushUp))
                .lineTo(sample2)
                .marker(robot.lift.goTo(liftPush3))
                .marker(1, -0.5, robot.lift.goTo(liftPush1))
                .pause(0.25)
                .marker(t -> robot.arm.setArm(armPushDown))
                .lineTo(drop2)
                .marker(robot.lift.goTo(liftPush2))
                .marker(1, -0.15, t -> robot.arm.setArm(armPushUp))
                .lineTo(sample3)
                .marker(robot.lift.goTo(liftPush3))
                .marker(1, -0.5, robot.lift.goTo(liftPush1))
                .pause(0.25)
                .marker(t -> robot.arm.setArm(armPushDown))
                .setTangent(0)
                .splineTo(preIntake, PI/2)
                .marker(robot.lift.goTo(liftPush2))
                .setMoveConstraints(intakeConstraints)
                .lineTo(intake.vec())
                .marker(new ParCommand(
                        FnCommand.once(t -> {
                            robot.arm.setArm(armGrab(0));
                            robot.arm.setClaw(false);}),
                        robot.lift.goBack(false, true)))
                .marker(1, -0.15, t -> robot.stateMachine.transition(GRABBED))
                .build(scheduler);
        Command traj2 = new TrajCommandBuilder(robot.drive, intake)
                .setMoveConstraints(toConstraints)
                .marker(0, 0.55, t -> robot.stateMachine.transition(SIDE_CHAMBER, liftHighSideChamber))
                .lineTo(specimen2.vec())
                .marker(1, -0.15, t -> robot.stateMachine.transition(EXTEND))
                .setVel(10)
                .setMoveConstraints(fromConstraints)
                .lineTo(preIntake.vec())
                .setMoveConstraints(intakeConstraints)
                .lineTo(intake.vec())
                .marker(1, -0.15, t -> robot.stateMachine.transition(GRABBED))
                .build(scheduler);
        Command traj3 = new TrajCommandBuilder(robot.drive, intake)
                .setMoveConstraints(toConstraints)
                .marker(0, 0.55, t -> robot.stateMachine.transition(SIDE_CHAMBER, liftHighSideChamber))
                .lineTo(specimen2.vec())
                .marker(1, -0.15, t -> robot.stateMachine.transition(EXTEND))
                .resetConstraints()
                .lineTo(preIntake.vec())
                .build(scheduler);
        scheduler.schedule(new SeqCommand(traj1, new RepeatCommand(traj2, 3), traj3, FnCommand.once(t -> end())));
        robot.drive.setPose(start);
    }
}