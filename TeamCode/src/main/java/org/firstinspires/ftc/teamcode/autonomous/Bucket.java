package org.firstinspires.ftc.teamcode.autonomous;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.MovementType.TURRET_FIRST;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.RobotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import static org.firstinspires.ftc.teamcode.hardware.Vision.*;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.LazyCommand;
import org.firstinspires.ftc.teamcode.command.LazySeqCommand;
import org.firstinspires.ftc.teamcode.command.ParCommand;
import org.firstinspires.ftc.teamcode.command.RepeatCommand;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.control.AsymProfile;
import org.firstinspires.ftc.teamcode.control.AsymProfile.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.MotionState;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.TrajCommandBuilder;
import org.firstinspires.ftc.teamcode.movement.Trajectory;
import org.firstinspires.ftc.teamcode.movement.Vec;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;
@Photon
@Autonomous(name = "Bucket")
public class Bucket extends AbstractAutonomous {
    private AsymConstraints grabConstraints = new AsymConstraints(70, 80, 40);
    private AsymConstraints dropConstraints = new AsymConstraints(70, 80, 40);
    private AsymConstraints cameraConstraints = new AsymConstraints(70, 80, 50);
    private Pose drop1 = new Pose(56.5, 56.5, -3*PI/4);
    private Pose drop2 = new Pose(59, 56, -2*PI/3);
    private Pose intake1 = new Pose(49, 35, -PI/2);
    private Pose intake2 = new Pose(59, 35, -PI/2);
    private Pose intake3 = new Pose(61, 35, -PI/4);
    private Pose park = new Pose(26, 12, -2.88);
    private Pose sample = new Pose(12, 0, 0);
    private double sampleXMin = -4;
    private double sampleXMax = 12;
    private double sampleYMin = -20;
    private double sampleYMax = 20;
    private Pose sub = park;
    private int config = 0;
    private Command dropTraj;
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
        Command traj1 = new ParCommand(
                        new TrajCommandBuilder(robot.drive, start)
                                .setMoveConstraints(grabConstraints)
                                .lineTo(drop1)
                                .marker(robot.drive.saveTraj())
                                .lineTo(intake1)
                                .marker(robot.drive.saveTraj())
                                .pause(0.15)
                                .lineTo(drop1)
                                .marker(robot.drive.saveTraj())
                                .pause(0.15)
                                .lineTo(intake2)
                                .marker(robot.drive.saveTraj())
                                .pause(0.15)
                                .lineTo(drop2)
                                .marker(robot.drive.saveTraj())
                                .pause(0.15)
                                .lineTo(intake3)
                                .marker(robot.drive.saveTraj())
                                .pause(0.15)
                                .lineTo(drop2)
                                .marker(robot.drive.saveTraj())
                                .pause(0.15)
                                .setMoveConstraints(cameraConstraints)
                                .setTangent(drop2.h)
                                .build(scheduler),
                        new SeqCommand(
                                robot.stateMachine.getTransition(GRABBED, BUCKET, liftHighBucket),
                                FnCommand.until(t -> t > robot.drive.curr.tf() - 0.15),
                                robot.stateMachine.getTransition(BUCKET, EXTEND, LiftPosition.inverse(new Vec(3, 0)), 0d),
                                FnCommand.until(t -> t > robot.drive.curr.tf()),
                                robot.stateMachine.getTransition(EXTEND, BUCKET, liftHighBucket),
                                FnCommand.until(t -> t > robot.drive.curr.tf()),
                                robot.stateMachine.getTransition(BUCKET, EXTEND, LiftPosition.inverse(new Vec(3, 0)), 0d),
                                FnCommand.until(t -> t > robot.drive.curr.tf()),
                                robot.stateMachine.getTransition(EXTEND, BUCKET, liftHighBucket),
                                FnCommand.until(t -> t > robot.drive.curr.tf()),
                                robot.stateMachine.getTransition(BUCKET, EXTEND, LiftPosition.inverse(new Vec(6, 0)), -PI/4),
                                FnCommand.until(t -> t > robot.drive.curr.tf() - 0.15),
                                robot.stateMachine.getTransition(EXTEND, BUCKET, liftHighBucket)));
        dropTraj = FnCommand.once(t -> {}, robot.drive);
        Command traj2 = new LazySeqCommand(
            () -> new ParCommand(
                new TrajCommandBuilder(robot.drive, drop2)
                        .pause(0.15)
                        .setMoveConstraints(cameraConstraints)
                        .setTangent(drop2.h)
                        .splineTo(sub.vec(), sub.h)
                        .marker(robot.drive.saveTraj())
                        .build(scheduler),
                new SeqCommand(
                    robot.stateMachine.getTransition(BUCKET, EXTEND, visionBucket.liftPos, 0d),
                    FnCommand.until(t -> t > robot.drive.curr.tf() - 0.15),
                    robot.vision.takeFrame(visionBucket),
                    FnCommand.once(t -> {
                        dropTraj = new ParCommand(
                                robot.stateMachine.getTransition(EXTEND, BUCKET, liftHighBucket),
                                new TrajCommandBuilder(robot.drive, sub)
                                        .pause(0.15)
                                        .setTangent(sub.h + PI)
                                        .setMoveConstraints(dropConstraints)
                                        .splineTo(drop2.vec(), drop2.h + PI)
                                        .build(scheduler));
                        List<Pose> valid = robot.vision.allPoses.stream()
                                .map(a -> dtToLift.inverse().add(sub.inverse().add(a)))
                                .filter(a -> a.x > liftXMin && a.x < liftXMax && abs(a.y) < liftYMax)
                                .sorted(Comparator.comparingDouble(a -> {
                                    LiftPosition pos = LiftPosition.inverse(a.vec());
                                    return new AsymProfile(liftDefaultConstraints, 0, new MotionState(pos.liftExt), new MotionState(12)).tf()
                                         + new AsymProfile(turretConstraints, 0, new MotionState(pos.turretAng), new MotionState(0)).tf();
                                }))
                                .collect(Collectors.toList());
                        if (!valid.isEmpty()) {
                            sample = valid.get(0);
                            List<Pose> nexts = robot.vision.allPoses.stream()
                                    .filter(a -> a.x > sampleXMin && a.x < sampleXMax
                                              && a.y > sampleYMin && a.y < sampleYMax
                                              && !sub.inverse().add(a).equals(sample))
                                    .map(this::sub)
                                    .sorted(Comparator.comparingDouble(a -> pow(park.y - a.y, 2) + 100 * pow(park.h - a.h, 2)))
                                    .collect(Collectors.toList());
                            if (!nexts.isEmpty()) {
                                sub = nexts.get(0);
                            } else {
                                sub = park;
                            }
                        } else {
                            sample = new Pose(12, 0, 0);
                            sub = park;}}),
                    new LazyCommand(() -> new ParCommand(
                        robot.lift.goTo(LiftPosition.inverse(sample.vec()), TURRET_FIRST, pivotCameraConstraints),
                        robot.arm.setGrab(sample.h, robot.lift))))),
            () -> dropTraj);
        Command traj3 = new ParCommand(
            new TrajCommandBuilder(robot.drive, drop2)
                    .setTangent(drop2.h)
                    .splineTo(park.vec(), park.h)
                    .build(scheduler),
            robot.stateMachine.getTransition(BUCKET, EXTEND, LiftPosition.inverse(new Vec(0, 0)), 0d));
        scheduler.schedule(new SeqCommand(
                traj1,
                config == 1 ? new RepeatCommand(traj2, 4) : null,
                traj3,
                FnCommand.once(t -> end())));
    }
    private Pose sub(Pose sample) {
        double angleY = sample.y + (park.x - sample.x) * tan(park.h);
        if (angleY < park.y) {
            return new Pose(park.x, angleY, park.h);
        } else {
            return new Pose(park.vec(), sample.y < park.y ? atan2(sample.y - park.y, sample.x - park.x) : PI);
        }
    }
}