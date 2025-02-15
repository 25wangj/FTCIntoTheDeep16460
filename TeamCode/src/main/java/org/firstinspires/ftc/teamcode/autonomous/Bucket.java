package org.firstinspires.ftc.teamcode.autonomous;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.autonomous.Chamber.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.robotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.ParCommand;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.control.AsymProfile.AsymConstraints;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.TrajCommandBuilder;

@Photon
@Autonomous(name = "Bucket")
public class Bucket extends AbstractAutonomous {
    private AsymConstraints grabConstraints = new AsymConstraints(20, 30, 20);
    private Pose start;
    private Pose mid;
    private Pose drop = new Pose(56, 56, -3*PI/4);
    private Pose specimen = new Pose(6.5, 39, -PI/2);
    private Pose intake1a = new Pose(48, 32, -PI/2);
    private Pose intake1b = new Pose(43, 30, -PI/4);
    private Pose intake2 = new Pose(58.5, 32, -PI/2);
    private Pose intake3 = new Pose(61, 32, -PI/4);
    private Pose park = new Pose(24, 12, -PI);
    private int config = 0;
    @Override
    public void chooseConfig() {
        while (config == 0 && !isStopRequested()) {
            telemetry.addLine("Press A for sample, B for park");
            telemetry.update();
            if (gamepad1.a) {
                config = 1;
            } else if (gamepad1.b) {
                config = 2;
            }
        }
    }
    @Override
    public void initAutonomous() {
        Command traj1;
        if (config == 1) {
            start = new Pose(31, 64, -PI);
            mid = intake1a;
            traj1 = new TrajCommandBuilder(robot.drive, start)
                    .lineTo(drop)
                    .marker(robot.stateMachine.getTransition(GRABBED, BUCKET, liftHighBucket))
                    .marker(1, -0.15, robot.stateMachine.getTransition(BUCKET, EXTEND,
                            new Pose(2, 0, 0)))
                    .setMoveConstraints(grabConstraints)
                    .lineTo(intake1a)
                    .marker(1, -0.15, new SeqCommand(
                            robot.stateMachine.getTransition(EXTEND, GRABBED),
                            robot.stateMachine.getTransition(GRABBED, BUCKET, liftHighBucket)))
                    .build(scheduler);
            telemetry.addData("Configuration", "Bucket Sample");
        } else {
            start = new Pose(6.5, 63, -PI/2);
            mid = intake1b;
            traj1 = new TrajCommandBuilder(robot.drive, start)
                    .setMoveConstraints(specConstraints)
                    .lineTo(specimen)
                    .marker(robot.stateMachine.getTransition(GRABBED, CHAMBER))
                    .marker(1, -0.15, robot.stateMachine.getTransition(CHAMBER, EXTEND,
                            new Pose(2, 0, -PI/4)))
                    .setTangent(PI/4)
                    .setMoveConstraints(sampleConstraints)
                    .splineTo(intake1b, -PI/4)
                    .marker(1, -0.15, new SeqCommand(
                            robot.stateMachine.getTransition(EXTEND, GRABBED),
                            robot.stateMachine.getTransition(GRABBED, BUCKET, liftHighBucket)))
                    .build(scheduler);
            telemetry.addData("Configuration", "Bucket Specimen");
        }
        Command traj2 = new TrajCommandBuilder(robot.drive, mid)
                .pause(0.15)
                .setMoveConstraints(grabConstraints)
                .lineTo(drop)
                .marker(1, -0.15, robot.stateMachine.getTransition(BUCKET, EXTEND,
                        new Pose(2, 0, 0)))
                .lineTo(intake2)
                .marker(1, -0.15, new SeqCommand(
                        robot.stateMachine.getTransition(EXTEND, GRABBED),
                        robot.stateMachine.getTransition(GRABBED, BUCKET, liftHighBucket)))
                .pause(0.15)
                .lineTo(drop)
                .marker(1, -0.15, robot.stateMachine.getTransition(BUCKET, EXTEND,
                        new Pose(5, 0, -PI/4)))
                .lineTo(intake3)
                .marker(1, -0.15, new SeqCommand(
                        robot.stateMachine.getTransition(EXTEND, GRABBED),
                        robot.stateMachine.getTransition(GRABBED, BUCKET, liftHighBucket)))
                .pause(0.15)
                .lineTo(drop)
                .marker(1, -0.15, new SeqCommand(
                        robot.stateMachine.getTransition(BUCKET, EXTEND, new Pose(0, 0, 0)),
                        robot.stateMachine.getTransition(EXTEND, GRABBED)))
                .resetConstraints()
                .splineTo(park.vec(), PI)
                .build(scheduler);
        scheduler.schedule(new SeqCommand(
                traj1,
                traj2,
                FnCommand.once(t -> end())));
        robot.drive.setPose(start);
    }
}
