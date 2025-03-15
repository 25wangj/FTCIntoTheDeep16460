package org.firstinspires.ftc.teamcode.autonomous;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.RobotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import static org.firstinspires.ftc.teamcode.hardware.Vision.Priority.*;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.LazyCommand;
import org.firstinspires.ftc.teamcode.command.LazySeqCommand;
import org.firstinspires.ftc.teamcode.command.ParCommand;
import org.firstinspires.ftc.teamcode.command.RepeatCommand;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.command.WaitCommand;
import org.firstinspires.ftc.teamcode.control.AsymProfile.AsymConstraints;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.TrajCommandBuilder;
import org.firstinspires.ftc.teamcode.movement.Trajectory;

@Photon
@Autonomous(name = "Bucket")
public class Bucket extends AbstractAutonomous {
    private AsymConstraints grabConstraints = new AsymConstraints(70, 80, 40);
    private AsymConstraints dropConstraints = new AsymConstraints(70, 80, 40);
    private AsymConstraints cameraConstraints = new AsymConstraints(70, 80, 50);
    private Pose drop1 = new Pose(56.5, 56.5, -3*PI/4);
    private Pose drop2 = new Pose(60, 52, -2*PI/3);
    private Pose intake1 = new Pose(49, 34, -PI/2);
    private Pose intake2 = new Pose(59, 34, -PI/2);
    private Pose intake3 = new Pose(61, 35, -PI/4);
    private Pose park = new Pose(25, 11, -2.88);
    private int config = 0;
    private Trajectory curr;
    private int i = 0;
    private int cycles = 4;
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
        start = new Pose(41, 64, -PI);
        Command traj1 = new SeqCommand(
                new ParCommand(
                        new TrajCommandBuilder(robot.drive, start)
                                .setMoveConstraints(grabConstraints)
                                .lineTo(drop1)
                                .marker(t -> curr = robot.drive.getTrajectory())
                                .pause(0.15)
                                .lineTo(intake1)
                                .build(scheduler),
                        new SeqCommand(
                                robot.stateMachine.getTransition(GRABBED, BUCKET, liftHighBucket),
                                FnCommand.until(t -> t > curr.tf() - 0.15),
                                robot.stateMachine.getTransition(BUCKET, EXTEND, new Pose(2, 0, 0)))),
                new ParCommand(
                        new TrajCommandBuilder(robot.drive, intake1)
                                .pause(0.15)
                                .lineTo(drop1)
                                .marker(t -> curr = robot.drive.getTrajectory())
                                .pause(0.15)
                                .lineTo(intake2)
                                .build(scheduler),
                        new SeqCommand(
                                robot.stateMachine.getTransition(EXTEND, BUCKET, liftHighBucket),
                                FnCommand.until(t -> t > curr.tf()),
                                robot.stateMachine.getTransition(BUCKET, EXTEND, new Pose(2, 0, 0)))),
                new ParCommand(
                        new TrajCommandBuilder(robot.drive, intake2)
                                .pause(0.15)
                                .setMoveConstraints(grabConstraints)
                                .lineTo(drop1)
                                .marker(t -> curr = robot.drive.getTrajectory())
                                .pause(0.15)
                                .lineTo(intake3)
                                .build(scheduler),
                        new SeqCommand(
                                robot.stateMachine.getTransition(EXTEND, BUCKET, liftHighBucket),
                                FnCommand.until(t -> t > curr.tf()),
                                robot.stateMachine.getTransition(BUCKET, EXTEND, new Pose(6, 0, -PI/4)))),
                new ParCommand(
                        new TrajCommandBuilder(robot.drive, intake3)
                                .setMoveConstraints(grabConstraints)
                                .pause(0.3)
                                .lineTo(drop2)
                                .marker(t -> curr = robot.drive.getTrajectory())
                                .pause(0.3)
                                .setMoveConstraints(cameraConstraints)
                                .setTangent(drop2.h)
                                .splineTo(park.vec(), park.h)
                                .build(scheduler),
                        new SeqCommand(
                                robot.stateMachine.getTransition(EXTEND, BUCKET, liftHighBucket),
                                FnCommand.until(t -> t > curr.tf()),
                                robot.stateMachine.getTransition(BUCKET, config == 1 ? CAMERA : GRABBED))));
        Command traj2;
        if (config == 1) {
            traj2 = new RepeatCommand(
                    new SeqCommand(
                            robot.stateMachine.getTransition(CAMERA, EXTEND, YELLOW_COLOR),
                            new ParCommand(
                                    FnCommand.once(t -> i++),
                                    new LazyCommand(() -> {
                                        TrajCommandBuilder traj = new TrajCommandBuilder(robot.drive, park)
                                                .pause(0.15)
                                                .setMoveConstraints(dropConstraints)
                                                .setTangent(park.h + PI)
                                                .splineTo(drop2.vec(), drop2.h + PI)
                                                .marker(t -> curr = robot.drive.getTrajectory());
                                        if (i < cycles) {
                                            traj.pause(0.15)
                                                    .setMoveConstraints(cameraConstraints)
                                                    .setTangent(drop2.h)
                                                    .splineTo(park.vec(), park.h);
                                        }
                                        return traj.build(scheduler);}),
                                    new LazySeqCommand(
                                            () -> robot.stateMachine.getTransition(EXTEND, BUCKET, liftHighBucket),
                                            () -> FnCommand.until(t -> t > (i < cycles ? curr.tf() : curr.tf() - 0.15)),
                                            () -> robot.stateMachine.getTransition(BUCKET, i < cycles ? CAMERA : GRABBED)))), cycles);
        } else {
            traj2 = null;
        }
        scheduler.schedule(new SeqCommand(
                traj1,
                traj2,
                FnCommand.once(t -> end())));
    }
}