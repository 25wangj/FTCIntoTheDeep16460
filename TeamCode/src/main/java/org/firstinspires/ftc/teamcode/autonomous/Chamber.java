package org.firstinspires.ftc.teamcode.autonomous;
import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Double.NaN;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.MovementType.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.RobotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import static org.firstinspires.ftc.teamcode.hardware.Vision.dtToLift;
import static org.firstinspires.ftc.teamcode.hardware.Vision.visionChamber;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.LazyCommand;
import org.firstinspires.ftc.teamcode.command.ParCommand;
import org.firstinspires.ftc.teamcode.command.RepeatCommand;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.command.WaitCommand;
import org.firstinspires.ftc.teamcode.control.AsymProfile;
import org.firstinspires.ftc.teamcode.control.AsymProfile.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.MotionState;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.TrajCommandBuilder;
import org.firstinspires.ftc.teamcode.movement.Vec;

import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

@Photon
@Autonomous(name = "Chamber")
public class Chamber extends AbstractAutonomous {
    public static final AsymConstraints sampleConstraints = new AsymConstraints(70, 80, 30);
    public static final AsymConstraints specToConstraints = new AsymConstraints(70, 80, 50);
    public static final AsymConstraints specBackConstraints = new AsymConstraints(70, 80, 40);
    private AsymConstraints turnToConstraints = new AsymConstraints(4, 8, 4);
    private AsymConstraints turnFromConstraints = new AsymConstraints(4, 8, 8);
    private Pose specimen1 = new Pose(-6.5, 39, -PI/2);
    private Pose sample1 = new Pose(-30.5, 42, -3*PI/4);
    private Pose sample2 = new Pose(-40.5, 42, -3*PI/4);
    private Pose sample3 = new Pose(-50.5, 42, -3*PI/4);
    private Pose drop1 = new Pose(-35.5, 46, -3.6);
    private Pose drop2 = new Pose(-45.5, 46, -3.6);
    public static final Pose wall1 = new Pose(-28, 64, -PI/2);
    public static final Pose wall2 = new Pose(-28, 63.5, -PI/2);
    public static final Pose specimen2 = new Pose(-5.5, 40, -PI/2);
    private double sampleX = 6;
    private double sampleYMax = 4;
    private Pose park = new Pose(-35, 60, -PI/2);
    private Command dropTraj;
    @Override
    public void initAutonomous() {
        start = new Pose(-6.5, 63, -PI/2);
        Command traj1 = new ParCommand(new TrajCommandBuilder(robot.drive, start)
                .lineTo(specimen1)
                .marker(robot.drive.saveTraj())
                .setMoveConstraints(sampleConstraints)
                .setTangent(5*PI/6)
                .setVel(NaN)
                .splineTo(new Pose(-15.5, 42, -3*PI/4), PI)
                .lineTo(sample1)
                .marker(robot.drive.saveTraj())
                .pause(0.15)
                .setTurnConstraints(turnFromConstraints)
                .lineTo(drop1)
                .marker(robot.drive.saveTraj())
                .setTurnConstraints(turnToConstraints)
                .lineTo(sample2)
                .marker(robot.drive.saveTraj())
                .pause(0.15)
                .setTurnConstraints(turnFromConstraints)
                .lineTo(drop2)
                .marker(robot.drive.saveTraj())
                .setTurnConstraints(turnToConstraints)
                .lineTo(sample3)
                .marker(robot.drive.saveTraj())
                .pause(0.15)
                .resetConstraints()
                .setMoveConstraints(specBackConstraints)
                .setTangent(0)
                .splineTo(wall2, PI/3)
                .pause(0.15)
                .build(scheduler),
            new SeqCommand(
                    robot.stateMachine.getTransition(GRABBED, CHAMBER),
                    FnCommand.until(t -> t > robot.drive.curr.tf() - 0.25),
                    robot.stateMachine.getTransition(CHAMBER, EXTEND,
                            LiftPosition.inverse(new Vec(0, 0)), PI/4, PIVOT_RETRACT, false),
                    FnCommand.until(t -> t > robot.drive.curr.tf() - 0.65),
                    robot.lift.goTo(LiftPosition.inverse(new Vec(18, 0)), PIVOT_FIRST),
                    FnCommand.until(t -> t > robot.drive.curr.tf()),
                    robot.stateMachine.getTransition(EXTEND, EXTEND_GRAB),
                    robot.lift.goTo(LiftPosition.inverse(new Vec(10, 0)), PIVOT_FIRST),
                    FnCommand.until(t -> t > robot.drive.curr.tf() - 0.4),
                    new ParCommand(
                        robot.lift.goTo(LiftPosition.inverse(new Vec(18, 0)), PIVOT_FIRST),
                        new SeqCommand(
                            FnCommand.until(t -> t > robot.lift.restTime() - 0.15),
                            robot.stateMachine.getTransition(EXTEND_GRAB, EXTEND))),
                    robot.lift.goTo(LiftPosition.inverse(new Vec(10, 0)), PIVOT_FIRST),
                    FnCommand.until(t -> t > robot.drive.curr.tf() - 0.5),
                    robot.lift.goTo(LiftPosition.inverse(new Vec(18, 0)), PIVOT_FIRST),
                    FnCommand.until(t -> t > robot.drive.curr.tf()),
                    robot.stateMachine.getTransition(EXTEND, EXTEND_GRAB),
                    robot.lift.goTo(LiftPosition.inverse(new Vec(10, 0)), PIVOT_FIRST),
                    FnCommand.until(t -> t > robot.drive.curr.tf() - 0.4),
                    new ParCommand(
                            robot.lift.goTo(LiftPosition.inverse(new Vec(18, 0)), PIVOT_FIRST),
                            new SeqCommand(
                                    FnCommand.until(t -> t > robot.lift.restTime() - 0.15),
                                    robot.stateMachine.getTransition(EXTEND_GRAB, EXTEND))),
                    robot.lift.goTo(LiftPosition.inverse(new Vec(10, 0)), PIVOT_FIRST),
                    FnCommand.until(t -> t > robot.drive.curr.tf() - 0.5),
                    robot.lift.goTo(LiftPosition.inverse(new Vec(18, 0)), PIVOT_FIRST),
                    FnCommand.until(t -> t > robot.drive.curr.tf()),
                    robot.stateMachine.getTransition(EXTEND, WALL)));
        dropTraj = FnCommand.once(t -> {}, robot.drive);
        Command traj2 = new SeqCommand(
                robot.stateMachine.getTransition(WALL, EXTEND, visionChamber.liftPos, 0d, TURRET_FIRST, true),
                new WaitCommand(0.15),
                robot.vision.takeFrame(visionChamber),
                FnCommand.once(t -> {
                    Pose sample = dtToLift.add(new Pose(sampleX + 8, 0, 0));
                    List<Pose> valid = robot.vision.colorPoses.stream()
                            .map(a -> dtToLift.inverse().add(specimen2.inverse().add(a)))
                            .filter(a -> a.x - sampleX > liftXMin && a.x - sampleX < liftXMax && abs(a.y) < sampleYMax + liftYMax)
                            .sorted(Comparator.comparingDouble(a -> pow(a.x - 8, 2) + 0.25 * pow(a.y, 2)))
                            .collect(Collectors.toList());
                    if (!valid.isEmpty()) {
                        sample = valid.get(0);
                    }
                    Pose sampleDrive = new Pose(sampleX, clip(sample.y, -sampleYMax, sampleYMax), 0);
                    Pose sampleLift = sampleDrive.inverse().add(sample);
                    sampleDrive = specimen2.add(sampleDrive);
                    dropTraj = new SeqCommand(
                        new ParCommand(
                            robot.lift.goTo(LiftPosition.inverse(sampleLift.vec()), PIVOT_RETRACT),
                            robot.arm.setGrab(sampleLift.h, robot.lift),
                            new TrajCommandBuilder(robot.drive, specimen2)
                                    .pause(0.5)
                                    .lineTo(sampleDrive)
                                    .build(scheduler)),
                        new ParCommand(
                                robot.stateMachine.getTransition(EXTEND, WALL),
                                new SeqCommand(
                                        FnCommand.until(t2 -> robot.lift.liftPos(t2).pivotAng != 0),
                                        new TrajCommandBuilder(robot.drive, sampleDrive)
                                                .setMoveConstraints(specBackConstraints)
                                                .lineTo(wall2)
                                                .pause(0.15)
                                                .build(scheduler))));}),
                new LazyCommand(() -> dropTraj));
        Command traj3 = new SeqCommand(
                robot.stateMachine.getTransition(WALL, EXTEND, visionChamber.liftPos, 0d, PIVOT_FIRST, true),
                new ParCommand(
                    robot.lift.goTo(LiftPosition.inverse(new Vec(0, 0)), PIVOT_RETRACT),
                    new TrajCommandBuilder(robot.drive, specimen2)
                            .lineTo(park)
                            .build(scheduler)));
        scheduler.schedule(new SeqCommand(
                traj1,
                traj2,
                new RepeatCommand(robot.stateMachine.getTransition(WALL, WALL), 3),
                traj3,
                FnCommand.once(t -> end())));
    }
}