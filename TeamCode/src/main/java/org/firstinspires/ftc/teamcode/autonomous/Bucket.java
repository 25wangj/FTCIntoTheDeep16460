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
@Autonomous(name = "Bucket")
public class Bucket extends AbstractAutonomous {
    private AsymConstraints grabConstraints = new AsymConstraints(70, 50, 30);
    private AsymConstraints dropConstraints = new AsymConstraints(70, 70, 30);
    private Pose drop1 = new Pose(56, 56, -3*PI/4);
    private Pose drop2 = new Pose(58, 52, -2*PI/3);
    private Pose intake1 = new Pose(48, 32, -PI/2);
    private Pose intake2 = new Pose(58, 32, -PI/2);
    private Pose intake3 = new Pose(60, 32, -PI/4);
    private Pose park = new Pose(24, 6, -2.88);
    private int config = 0;
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
                                .setMoveConstraints(grabConstraints)
                                .lineTo(drop1)
                                .build(scheduler),
                        robot.stateMachine.getTransition(GRABBED, BUCKET, liftHighBucket)),
                new ParCommand(
                        new TrajCommandBuilder(robot.drive, drop1)
                                .pause(0.15)
                                .setMoveConstraints(grabConstraints)
                                .lineTo(intake1)
                                .build(scheduler),
                        robot.stateMachine.getTransition(BUCKET, EXTEND, new Pose(2, 0, 0))),
                new ParCommand(
                        new TrajCommandBuilder(robot.drive, intake1)
                                .pause(0.15)
                                .setMoveConstraints(grabConstraints)
                                .lineTo(drop1)
                                .build(scheduler),
                        robot.stateMachine.getTransition(EXTEND, BUCKET, liftHighBucket)),
                new ParCommand(
                        new TrajCommandBuilder(robot.drive, drop1)
                                .pause(0.15)
                                .setMoveConstraints(grabConstraints)
                                .lineTo(intake2)
                                .build(scheduler),
                        robot.stateMachine.getTransition(BUCKET, EXTEND, new Pose(2, 0, 0))),
                new ParCommand(
                        new TrajCommandBuilder(robot.drive, intake2)
                                .pause(0.15)
                                .setMoveConstraints(grabConstraints)
                                .lineTo(drop1)
                                .build(scheduler),
                        robot.stateMachine.getTransition(EXTEND, BUCKET, liftHighBucket)),
                new ParCommand(
                        new TrajCommandBuilder(robot.drive, drop1)
                                .pause(0.15)
                                .setMoveConstraints(grabConstraints)
                                .lineTo(intake3)
                                .build(scheduler),
                        robot.stateMachine.getTransition(BUCKET, EXTEND, new Pose(6, 0, -PI/4))),
                new ParCommand(
                        new TrajCommandBuilder(robot.drive, intake3)
                                .pause(0.15)
                                .setMoveConstraints(grabConstraints)
                                .lineTo(drop2)
                                .build(scheduler),
                        robot.stateMachine.getTransition(EXTEND, BUCKET, liftHighBucket)));
        Command traj2;
        if (config == 1) {
            traj2 = new SeqCommand(
                    new RepeatCommand(
                            new SeqCommand(
                                    new ParCommand(
                                            robot.stateMachine.getTransition(BUCKET, CAMERA),
                                            new TrajCommandBuilder(robot.drive, drop2)
                                                    .pause(0.15)
                                                    .setTangent(drop2.h)
                                                    .splineTo(park.vec(), park.h)
                                                    .build(scheduler)),
                                    robot.stateMachine.getTransition(CAMERA, EXTEND, YELLOW_COLOR),
                                    new ParCommand(
                                            robot.stateMachine.getTransition(EXTEND, BUCKET, liftHighBucket),
                                            new TrajCommandBuilder(robot.drive, park)
                                                    .pause(0.25)
                                                    .setMoveConstraints(dropConstraints)
                                                    .setTangent(park.h + PI)
                                                    .splineTo(drop2.vec(), drop2.h + PI)
                                                    .build(scheduler))), 3),
                    new ParCommand(
                            robot.stateMachine.getTransition(BUCKET, GRABBED),
                            new TrajCommandBuilder(robot.drive, drop2)
                                    .pause(0.15)
                                    .setTangent(drop2.h)
                                    .splineTo(park.vec(), park.h)
                                    .build(scheduler)));
        } else {
            traj2 = new ParCommand(
                    robot.stateMachine.getTransition(BUCKET, GRABBED),
                    new TrajCommandBuilder(robot.drive, drop2)
                            .pause(0.15)
                            .setTangent(drop2.h)
                            .splineTo(park, PI)
                            .build(scheduler));
        }
        scheduler.schedule(new SeqCommand(
                traj1,
                traj2,
                FnCommand.once(t -> end())));
    }
}