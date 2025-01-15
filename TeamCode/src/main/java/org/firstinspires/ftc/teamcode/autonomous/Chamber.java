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
import org.firstinspires.ftc.teamcode.command.WaitCommand;
import org.firstinspires.ftc.teamcode.control.AsymProfile.AsymConstraints;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.TrajCommandBuilder;
import org.firstinspires.ftc.teamcode.movement.Vec;

@Photon
@Autonomous(name = "Chamber")
public class Chamber extends AbstractAutonomous {
    private AsymConstraints pushConstraints = new AsymConstraints(30, 40, 40);
    private AsymConstraints toConstraints = new AsymConstraints(30, 30, 30);
    private AsymConstraints fromConstraints = new AsymConstraints(70, 70, 30);
    private AsymConstraints pushTurnConstraints = new AsymConstraints(4, 16, 4);
    private Pose start = new Pose(-6.5, 63, PI/2);
    private Pose specimen1 = new Pose(-6.5, 31, PI/2);
    private Pose specimen2 = new Pose(-6, 30, 5*PI/6);
    private Pose sample1 = new Pose(-32, 42, 5*PI/4);
    private Pose sample2 = new Pose(-42.5, 42, 5*PI/4);
    private Pose sample3 = new Pose(-53, 42, 5*PI/4);
    private Pose drop1 = new Pose(-31.5, 46, 5*PI/6);
    private Pose drop2 = new Pose(-42, 46, 5*PI/6);
    private Pose intake = new Pose(-37, 50, 5*PI/6);
    private Pose park = new Pose(-40, 56, 5*PI/6);
    @Override
    public void initAutonomous() {
        Command traj1 = new TrajCommandBuilder(robot.drive, start)
                .lineTo(specimen1)
                //.marker(t -> robot.stateMachine.transition(BACK_CHAMBER, liftBackChamber))
                .marker(1, -0.15, t -> robot.stateMachine.transition(EXTEND, new Pose(18, 0, PI/4)))
                .setMoveConstraints(pushConstraints)
                .splineTo(sample1, sample1.h)
                .marker(1, -0.15, t -> robot.stateMachine.transition(EXTEND_GRAB))
                .pause(0.15)
                .setTurnConstraints(pushTurnConstraints)
                .lineTo(drop1)
                .marker(1, -0.15, t -> robot.stateMachine.transition(EXTEND))
                .lineTo(sample2)
                .marker(1, -0.15, t -> robot.stateMachine.transition(EXTEND_GRAB))
                .pause(0.15)
                .setTurnConstraints(pushTurnConstraints)
                .lineTo(drop2)
                .marker(1, -0.15, t -> robot.stateMachine.transition(EXTEND))
                .lineTo(sample3)
                .marker(1, -0.15, t -> robot.stateMachine.transition(EXTEND_GRAB))
                .pause(0.15)
                .setTangent(0)
                .splineTo(intake, PI/2)
                .marker(1, -0.5, robot.arm.setGrab(PI/6, robot.lift))
                .marker(1, -0.15, new SeqCommand(
                    new WaitCommand(t -> robot.stateMachine.transition(EXTEND), 0.15),
                    robot.lift.goTo(LiftPosition.inverse(new Vec(6, 0)))))
                .pause(2)
                .marker(1, -0.3, t -> robot.stateMachine.transition(GRABBED))
                .build(scheduler);
        /*Command traj2 = new TrajCommandBuilder(robot.drive, intake)
                .setMoveConstraints(toConstraints)
                //.marker(0, 0.4, t -> robot.stateMachine.transition(SIDE_CHAMBER, liftSideChamber))
                .lineTo(specimen2.vec())
                .marker(1, -0.15, t -> robot.stateMachine.transition(EXTEND, new Pose(6, 0, PI/6)))
                .setMoveConstraints(fromConstraints)
                .lineTo(intake.vec())
                .pause(0.5)
                .marker(1, -0.3, t -> robot.stateMachine.transition(GRABBED))
                .build(scheduler);*/
        /*Command traj3 = new TrajCommandBuilder(robot.drive, intake)
                .setMoveConstraints(toConstraints)
                //.marker(0, 0.4, t -> robot.stateMachine.transition(SIDE_CHAMBER, liftSideChamber))
                .lineTo(specimen2.vec())
                .marker(1, -0.15, t -> robot.stateMachine.transition(EXTEND, new Pose(0, 0, 0)))
                .resetConstraints()
                .lineTo(park.vec())
                .marker(1, -0.15, t -> robot.stateMachine.transition(GRABBED))
                .build(scheduler);*/
        scheduler.schedule(new SeqCommand(traj1/*, new RepeatCommand(traj2, 3), traj3, FnCommand.once(t -> end())*/));
        robot.drive.setPose(start);
    }
}