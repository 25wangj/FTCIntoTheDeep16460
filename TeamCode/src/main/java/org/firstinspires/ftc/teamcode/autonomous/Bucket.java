package org.firstinspires.ftc.teamcode.autonomous;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.RobotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import static org.firstinspires.ftc.teamcode.hardware.Vision.Priority.*;
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
@Autonomous(name = "BucketNew")
public class Bucket extends AbstractAutonomous {
    private AsymConstraints slowConstraints = new AsymConstraints(70, 70, 30);
    private Pose drop = new Pose(56, 56, -3*PI/4);
    private Pose intake1 = new Pose(48, 32, -PI/2);
    private Pose intake2 = new Pose(58, 32, -PI/2);
    private Pose intake3 = new Pose(60, 32, -PI/4);
    private Pose park = new Pose(24, 12, -PI);
    private int config =    0;
    @Override
    public void chooseConfig() {
        while (config == 0 && !isStopRequested()) {
            telemetry.addLine("Press X for cycles, Y for park");
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
        start = new Pose(39.5, 64, -PI);
        Command traj1 = new SeqCommand(
                new ParCommand(
                        new TrajCommandBuilder(robot.drive, start)
                                .lineTo(drop)
                                .build(scheduler),
                        robot.stateMachine.getTransition(GRABBED, BUCKET, liftHighBucket)),
                new ParCommand(
                        new TrajCommandBuilder(robot.drive, drop)
                                .pause(0.15)
                                .setMoveConstraints(slowConstraints)
                                .lineTo(intake1)
                                .build(scheduler),
                        robot.stateMachine.getTransition(BUCKET, EXTEND, new Pose(2, 0, 0))),
                new ParCommand(
                        new TrajCommandBuilder(robot.drive, intake1)
                                .pause(0.15)
                                .lineTo(drop)
                                .build(scheduler),
                        robot.stateMachine.getTransition(EXTEND, BUCKET, liftHighBucket)),
                new ParCommand(
                        new TrajCommandBuilder(robot.drive, drop)
                                .pause(0.15)
                                .setMoveConstraints(slowConstraints)
                                .lineTo(intake2)
                                .build(scheduler),
                        robot.stateMachine.getTransition(BUCKET, EXTEND, new Pose(2, 0, 0))),
                new ParCommand(
                        new TrajCommandBuilder(robot.drive, intake2)
                                .pause(0.15)
                                .lineTo(drop)
                                .build(scheduler),
                        robot.stateMachine.getTransition(EXTEND, BUCKET, liftHighBucket)),
                new ParCommand(
                        new TrajCommandBuilder(robot.drive, drop)
                                .pause(0.15)
                                .setMoveConstraints(slowConstraints)
                                .lineTo(intake3)
                                .build(scheduler),
                        robot.stateMachine.getTransition(BUCKET, EXTEND, new Pose(6, 0, -PI/4))),
                new ParCommand(
                        new TrajCommandBuilder(robot.drive, intake3)
                                .pause(0.15)
                                .lineTo(drop)
                                .build(scheduler),
                        robot.stateMachine.getTransition(EXTEND, BUCKET, liftHighBucket)));
        Command traj2 = new ParCommand(
                robot.stateMachine.getTransition(BUCKET, GRABBED),
                new TrajCommandBuilder(robot.drive, drop)
                        .splineTo(park, PI)
                        .build(scheduler));
        if (config == 1) {
            traj2 = new SeqCommand(
                new RepeatCommand(
                    new SeqCommand(
                        new ParCommand(
                            robot.stateMachine.getTransition(BUCKET, CAMERA),
                            new TrajCommandBuilder(robot.drive, drop)
                                    .pause(0.15)
                                    .setMoveConstraints(slowConstraints)
                                    .setTangent(-2*PI/3)
                                    .splineTo(park, PI)
                                    .pause(0.25)
                                    .build(scheduler)),
                        robot.stateMachine.getTransition(CAMERA, EXTEND, YELLOW_COLOR),
                        new ParCommand(
                            robot.stateMachine.getTransition(EXTEND, BUCKET, liftHighBucket),
                            new TrajCommandBuilder(robot.drive, park)
                                    .pause(0.15)
                                    .setMoveConstraints(slowConstraints)
                                    .setTangent(0)
                                    .splineTo(drop, PI/3)
                                    .build(scheduler))), 3),
                traj2);
        }
        scheduler.schedule(new SeqCommand(
                traj1,
                traj2,
                FnCommand.once(t -> end())));
    }
}
