package org.firstinspires.ftc.teamcode.autonomous;
import static java.lang.Double.NaN;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.MovementType.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.RobotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.FnCommand;
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
    public static final AsymConstraints sampleConstraints = new AsymConstraints(70, 70, 30);
    public static final AsymConstraints specConstraints = new AsymConstraints(70, 70, 40);
    private AsymConstraints sampleTurnConstraints = new AsymConstraints(4, 8, 4);
    private Pose specimen1 = new Pose(-6.5, 39, -PI/2);
    private Pose sample1 = new Pose(-32, 42, -3*PI/4);
    private Pose sample2 = new Pose(-42.5, 42, -3*PI/4);
    private Pose sample3 = new Pose(-52.5, 42, -3*PI/4);
    private Pose drop1 = new Pose(-37.5, 48, -7*PI/6);
    private Pose drop2 = new Pose(-47.5, 48, -7*PI/6);
    public static final Pose wall = new Pose(-29, 64, -PI/2);
    public static final Pose specimen2 = new Pose(-4.5, 40, -PI/2);
    private Pose bucket = new Pose(58, 56, -3*PI/4);
    private Pose park = new Pose(-35, 60, -PI);
    private int config = 0;
    @Override
    public void chooseConfig() {
        while (config == 0 && !isStopRequested()) {
            telemetry.addLine("Press X for sample, Y for park");
            telemetry.update();
            if (gamepad1.x) {
                config = 1;
            } else if (gamepad1.y) {
                config = 2;
            }
        }
    }
    @Override
    public void initAutonomous() {
        start = new Pose(-6.5, 63, -PI/2);
        Command traj1 = new TrajCommandBuilder(robot.drive, start)
                .setMoveConstraints(specConstraints)
                .lineTo(specimen1)
                .marker(robot.stateMachine.getTransition(GRABBED, CHAMBER))
                .marker(1, -0.15, robot.stateMachine.getTransition(CHAMBER, EXTEND,
                        new Pose(0, 0, PI/4)))
                .setMoveConstraints(sampleConstraints)
                .lineTo(sample1)
                .marker(1, -0.75, robot.lift.goTo(LiftPosition.inverse(new Vec(18, 0)), PIVOT_FIRST))
                .marker(1, -0.15, robot.stateMachine.getTransition(EXTEND, EXTEND_GRAB))
                .pause(0.15)
                .setTurnConstraints(sampleTurnConstraints)
                .lineTo(drop1)
                .marker(robot.lift.goTo(LiftPosition.inverse(new Vec(10, 0)), PIVOT_FIRST))
                .marker(1, -0.5, robot.lift.goTo(LiftPosition.inverse(new Vec(18, 0)), PIVOT_FIRST))
                .marker(1, -0.15, robot.stateMachine.getTransition(EXTEND_GRAB, EXTEND))
                .lineTo(sample2)
                .marker(robot.lift.goTo(LiftPosition.inverse(new Vec(10, 0)), PIVOT_FIRST))
                .marker(1, -0.55, robot.lift.goTo(LiftPosition.inverse(new Vec(18, 0)), PIVOT_FIRST))
                .marker(1, -0.15, robot.stateMachine.getTransition(EXTEND, EXTEND_GRAB))
                .pause(0.15)
                .lineTo(drop2)
                .marker(robot.lift.goTo(LiftPosition.inverse(new Vec(10, 0)), PIVOT_FIRST))
                .marker(1, -0.5, robot.lift.goTo(LiftPosition.inverse(new Vec(18, 0)), PIVOT_FIRST))
                .marker(1, -0.15, robot.stateMachine.getTransition(EXTEND_GRAB, EXTEND))
                .lineTo(sample3)
                .marker(robot.lift.goTo(LiftPosition.inverse(new Vec(10, 0)), PIVOT_FIRST))
                .marker(1, -0.55, robot.lift.goTo(LiftPosition.inverse(new Vec(18, 0)), PIVOT_FIRST))
                .marker(1, -0.15, new SeqCommand(
                        robot.stateMachine.getTransition(EXTEND, GRABBED),
                        robot.stateMachine.getTransition(GRABBED, WALL)))
                .pause(0.15)
                .setTangent(0)
                .splineTo(wall, PI/2)
                .build(scheduler);
        Command traj2;
        if (config == 1) {
            traj2 = new SeqCommand(
                    new WaitCommand(t -> {
                        robot.drive.setTrajectory(null);
                        robot.drive.setPowers(new Vec(-0.3, 0), 0);}, 0.25),
                    new WaitCommand(t -> robot.stateMachine.transition(WALL, GRABBED), 0.3),
                    new TrajCommandBuilder(robot.drive, wall)
                        .setMoveConstraints(sampleConstraints)
                        .setVel(NaN)
                        .lineTo(new Pose(40, 54, -3*PI/4))
                        .lineTo(bucket)
                        .marker(1, -1.75, robot.stateMachine.getTransition(GRABBED, BUCKET, liftHighBucket))
                        .marker(1, -0.15, robot.stateMachine.getTransition(BUCKET, GRABBED))
                        .setMoveConstraints(new AsymConstraints(100, 70, 100))
                        .setTangent(-5*PI/6)
                        .setVel(NaN)
                        .splineTo(new Pose(40, 60, -PI), 0)
                        .lineTo(park)
                        .build(scheduler));
            telemetry.addData("Configuration", "Chamber Sample");
        } else {
            traj2 = new SeqCommand(
                    new WaitCommand(t -> {
                        robot.drive.setTrajectory(null);
                        robot.drive.setPowers(new Vec(-0.3, 0), 0);}, 0.25),
                    new WaitCommand(t -> robot.stateMachine.transition(WALL, GRABBED), 0.3),
                    new TrajCommandBuilder(robot.drive, wall)
                    .marker(t -> robot.drive.setPose(wall))
                    .lineTo(park)
                    .build(scheduler));
            telemetry.addData("Configuration", "Chamber Park");
        }
        scheduler.schedule(new SeqCommand(
                traj1,
                new RepeatCommand(robot.stateMachine.getTransition(WALL, WALL, 0.25), 4),
                traj2,
                FnCommand.once(t -> end())));
    }
}