package org.firstinspires.ftc.teamcode.autonomous;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.robotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.control.AsymProfile.AsymConstraints;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.TrajCommandBuilder;
import org.firstinspires.ftc.teamcode.movement.Vec;

@Photon
@Autonomous(name = "Bucket")
public class Bucket extends AbstractAutonomous {
    private AsymConstraints grabConstraints = new AsymConstraints(30, 40, 30);
    private Pose start;
    private Pose drop = new Pose(56, 56, -3*PI/4);
    private Pose specimen = new Pose(11, 31, PI/2);
    private Pose intake1a = new Pose(48, 31, -PI/2);
    private Pose intake1b = new Pose(43, 29, -PI/4);
    private Pose intake2 = new Pose(58, 31, -PI/2);
    private Pose intake3 = new Pose(61, 31, -PI/4);
    private Pose park = new Pose(24, 12, -PI);
    private int config = 0;
    @Override
    public void initAutonomous() {
        while (config == 0 && !isStopRequested()) {
            telemetry.addLine("Press A for sample, B for specimen");
            telemetry.update();
            if (gamepad1.a) {
                config = 1;
            } else if (gamepad1.b) {
                config = 2;
            }
        }
        Command traj1;
        if (config == 1) {
            start = new Pose(31, 64, -PI);
            traj1 = new TrajCommandBuilder(robot.drive, start)
                    .lineTo(drop)
                    .marker(t -> robot.stateMachine.transition(BUCKET, liftHighBucket))
                    .marker(1, -0.15, t -> robot.stateMachine.transition(EXTEND, new Pose(2, 0, 0)))
                    .setMoveConstraints(grabConstraints)
                    .lineTo(intake1a)
                    .marker(1, -0.15, t -> robot.stateMachine.transition(GRABBED))
                    .pause(0.15)
                    .lineTo(drop)
                    .marker(0, 0.4, t -> robot.stateMachine.transition(BUCKET, liftHighBucket))
                    .marker(1, -0.15, t -> robot.stateMachine.transition(EXTEND, new Pose(2, 0, 0)))
                    .build(scheduler);
            telemetry.addData("Configuration", "Bucket Sample");
        } else {
            start = new Pose(6.5, 63, PI/2);
            traj1 = new TrajCommandBuilder(robot.drive, start)
                    .lineTo(specimen)
                    //.marker(t -> robot.stateMachine.transition(BACK_CHAMBER, liftBackChamber))
                    .marker(1, -0.15, t -> robot.stateMachine.transition(EXTEND, new Pose(2, 0, -PI/4)))
                    .setTangent(PI/4)
                    .setMoveConstraints(new AsymConstraints(70, 70, 30))
                    .splineTo(intake1b, -PI/4)
                    .marker(1, -0.15, t -> robot.stateMachine.transition(GRABBED))
                    .pause(0.15)
                    .setMoveConstraints(grabConstraints)
                    .lineTo(drop)
                    .marker(0, 0.4, t -> robot.stateMachine.transition(BUCKET, liftHighBucket))
                    .marker(1, -0.15, t -> robot.stateMachine.transition(EXTEND, new Pose(2, 0, 0)))
                    .build(scheduler);
            telemetry.addData("Configuration", "Bucket Specimen");
        }
        Command traj2 = new TrajCommandBuilder(robot.drive, drop)
                .setMoveConstraints(grabConstraints)
                .lineTo(intake2)
                .marker(1, -0.15, t -> robot.stateMachine.transition(GRABBED))
                .pause(0.15)
                .lineTo(drop)
                .marker(0, 0.4, t -> robot.stateMachine.transition(BUCKET, liftHighBucket))
                .marker(1, -0.15, t -> robot.stateMachine.transition(EXTEND, new Pose(5, 0, -PI/4)))
                .lineTo(intake3)
                .marker(1, -0.15, t -> robot.stateMachine.transition(GRABBED))
                .pause(0.15)
                .lineTo(drop)
                .marker(0, 0.5, t -> robot.stateMachine.transition(BUCKET, liftHighBucket))
                .marker(1, -0.15, t -> robot.stateMachine.transition(EXTEND, new Pose(0, 0, 0)))
                .resetConstraints()
                .splineTo(park.vec(), PI)
                .marker(1, -0.15, t -> robot.stateMachine.transition(GRABBED))
                .build(scheduler);
        scheduler.schedule(new SeqCommand(traj1, traj2, FnCommand.once(t -> end())));
        robot.drive.setPose(start);
    }
}
