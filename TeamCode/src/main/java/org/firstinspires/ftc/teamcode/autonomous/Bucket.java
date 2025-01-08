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
    private AsymConstraints slowConstraints = new AsymConstraints(45, 60, 40);
    private AsymConstraints dropConstraints = new AsymConstraints(30, 40, 40);
    private AsymConstraints grabConstraints = new AsymConstraints(10, 30, 30);
    private Pose start;
    private Pose drop = new Pose(56, 56, -3*PI/4);
    private Pose specimen = new Pose(11, 31, PI/2);
    private Pose intake1 = new Pose(48, 28, -PI/2);
    private Pose intake2 = new Pose(58.5, 28, -PI/2);
    private Pose intake3 = new Pose(64, 18, 0);
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
                    .setMoveConstraints(slowConstraints)
                    .lineTo(drop)
                    .marker(t -> robot.stateMachine.transition(BUCKET, liftHighBucket))
                    .marker(t -> robot.stateMachine.transition(EXTEND, new Pose(6, 0, 0)))
                    .setTangent(-3*PI/4)
                    .setVel(10)
                    .splineTo(new Pose(48, 38, -PI/2), -PI/2)
                    .setMoveConstraints(grabConstraints)
                    .lineTo(intake1.vec())
                    .marker(1, -0.15, t -> robot.stateMachine.transition(GRABBED))
                    .setMoveConstraints(dropConstraints)
                    .lineTo(drop)
                    .marker(0, 0.6, t -> robot.stateMachine.transition(BUCKET, liftHighBucket))
                    .build(scheduler);
            telemetry.addData("Configuration", "Bucket Sample");
        } else {
            start = new Pose(6.5, 63, PI/2);
            traj1 = new TrajCommandBuilder(robot.drive, start)
                    .lineTo(specimen)
                    .marker(t -> robot.stateMachine.transition(BACK_CHAMBER, liftHighBackChamber))
                    .marker(1, -0.15, t -> robot.stateMachine.transition(EXTEND))
                    .pause(0.25)
                    .setTangent(PI/4)
                    .setVel(10)
                    .splineTo(new Pose(44, 36, -1.3), -1.3)
                    .setMoveConstraints(grabConstraints)
                    .lineTo(intake1.vec())
                    .marker(1, -0.15, t -> robot.stateMachine.transition(GRABBED))
                    .setMoveConstraints(dropConstraints)
                    .lineTo(drop)
                    .marker(0, 0.55, t -> robot.stateMachine.transition(BUCKET, liftHighBucket))
                    .build(scheduler);
            telemetry.addData("Configuration", "Bucket Specimen");
        }
        Command traj2 = new TrajCommandBuilder(robot.drive, drop)
                .pause(0.25)
                .marker(t -> robot.stateMachine.transition(EXTEND))
                .setMoveConstraints(slowConstraints)
                .setTangent(-1.5)
                .setVel(10)
                .splineTo(new Pose(58, 38, -PI/2), -PI/2)
                .setMoveConstraints(grabConstraints)
                .lineTo(intake2)
                .marker(1, -0.15, t -> robot.stateMachine.transition(GRABBED))
                .setTangent(PI/2)
                .setMoveConstraints(dropConstraints)
                .lineTo(drop)
                .marker(0, 0.55, t -> robot.stateMachine.transition(BUCKET, liftHighBucket))
                .pause(0.25)
                .marker(t -> robot.stateMachine.transition(EXTEND))
                .setMoveConstraints(slowConstraints)
                .setTangent(-1.4)
                .setVel(10)
                .splineTo(new Pose(62, 32, -0.6), -PI/2)
                .setMoveConstraints(grabConstraints)
                .setVel(10)
                .lineTo(new Vec(62, 24))
                .lineTo(intake3)
                .marker(1, -0.15, t -> robot.stateMachine.transition(GRABBED))
                .setMoveConstraints(dropConstraints)
                .lineTo(drop)
                .marker(0, 0.55, t -> robot.stateMachine.transition(BUCKET, liftHighBucket))
                .pause(0.25)
                .marker(t -> robot.stateMachine.transition(EXTEND))
                .resetConstraints()
                .splineTo(park.vec(), PI)
                .build(scheduler);
        scheduler.schedule(new SeqCommand(traj1, traj2, FnCommand.once(t -> end())));
        robot.drive.setPose(start);
    }
}
