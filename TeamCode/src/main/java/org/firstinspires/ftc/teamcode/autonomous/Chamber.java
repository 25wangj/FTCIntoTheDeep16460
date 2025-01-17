package org.firstinspires.ftc.teamcode.autonomous;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.robotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;

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
import org.firstinspires.ftc.teamcode.movement.Vec;

@Photon
@Autonomous(name = "Chamber")
public class Chamber extends AbstractAutonomous {
    private AsymConstraints sampleConstraints = new AsymConstraints(60, 70, 30);
    private AsymConstraints sampleTurnConstraints = new AsymConstraints(4, 8, 4);
    private AsymConstraints specConstraints = new AsymConstraints(60, 70, 30);
    private Pose start = new Pose(-6.5, 63, -PI/2);
    private Pose specimen1 = new Pose(-6.5, 39, -PI/2);
    private Pose specimen2 = new Pose(-4.5, 40, -PI/2);
    private Pose sample1 = new Pose(-32, 42, -3*PI/4);
    private Pose sample2 = new Pose(-42.5, 42, -3*PI/4);
    private Pose sample3 = new Pose(-53, 42, -3*PI/4);
    private Pose drop1 = new Pose(-31.5, 46, -7*PI/6);
    private Pose drop2 = new Pose(-42, 46, -7*PI/6);
    private Pose wall = new Pose(-29, 64, -PI/2);
    private Pose bucket = new Pose(56, 56, -3*PI/4);
    @Override
    public void initAutonomous() {
        Command traj1 = new TrajCommandBuilder(robot.drive, start)
                .lineTo(specimen1)
                .marker(robot.stateMachine.getTransition(GRABBED, CHAMBER))
                .marker(1, -0.15, robot.stateMachine.getTransition(CHAMBER, EXTEND,
                        new Pose(18, 0, PI/4)))
                .setMoveConstraints(sampleConstraints)
                .lineTo(sample1)
                .marker(1, -0.15, robot.stateMachine.getTransition(EXTEND, EXTEND_GRAB))
                .pause(0.15)
                .setTurnConstraints(sampleTurnConstraints)
                .lineTo(drop1)
                .marker(1, -0.15, robot.stateMachine.getTransition(EXTEND_GRAB, EXTEND))
                .lineTo(sample2)
                .marker(1, -0.15, robot.stateMachine.getTransition(EXTEND, EXTEND_GRAB))
                .pause(0.15)
                .lineTo(drop2)
                .marker(1, -0.15, robot.stateMachine.getTransition(EXTEND_GRAB, EXTEND))
                .marker(1, 0, robot.lift.goTo(LiftPosition.inverse(new Vec(14, 0))))
                .lineTo(sample3)
                .marker(1, -0.5, robot.lift.goTo(LiftPosition.inverse(new Vec(18, 0))))
                .marker(1, -0.15, new SeqCommand(
                        robot.stateMachine.getTransition(EXTEND, GRABBED),
                        robot.stateMachine.getTransition(GRABBED, WALL)))
                .pause(0.15)
                .setTangent(0)
                .splineTo(wall, PI/2)
                .build(scheduler);
        Command traj2 = new TrajCommandBuilder(robot.drive, wall)
                .setMoveConstraints(specConstraints)
                .pause(0.3)
                .marker(robot.stateMachine.getTransition(WALL, CHAMBER))
                .lineTo(specimen2)
                .marker(1, -0.15, robot.stateMachine.getTransition(CHAMBER, WALL))
                .lineTo(wall)
                .build(scheduler);
        Command traj3 = new TrajCommandBuilder(robot.drive, wall)
                .pause(0.3)
                .marker(new SeqCommand(
                        robot.stateMachine.getTransition(WALL, GRABBED),
                        robot.stateMachine.getTransition(GRABBED, BUCKET, liftHighBucket)))
                .setMoveConstraints(new AsymConstraints(70, 70, 30))
                .lineTo(bucket)
                .marker(1, -0.15, new SeqCommand(
                        robot.stateMachine.getTransition(BUCKET, EXTEND, new Pose(0, 0, 0)),
                        robot.stateMachine.getTransition(EXTEND, GRABBED)))
                .pause(1)
                .build(scheduler);
        scheduler.schedule(new SeqCommand(
                traj1,
                new RepeatCommand(traj2, 4),
                traj3,
                FnCommand.once(t -> end())));
        robot.drive.setPose(start);
    }
}