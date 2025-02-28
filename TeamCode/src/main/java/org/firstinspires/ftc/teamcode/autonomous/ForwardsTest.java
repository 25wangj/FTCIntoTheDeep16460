package org.firstinspires.ftc.teamcode.autonomous;
import static java.lang.Math.*;
import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.RepeatCommand;
import org.firstinspires.ftc.teamcode.control.AsymProfile.AsymConstraints;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.TrajCommandBuilder;
import org.firstinspires.ftc.teamcode.movement.TrajectoryState;
import org.firstinspires.ftc.teamcode.movement.Twist;
import org.firstinspires.ftc.teamcode.movement.Vec;
@Config
@Photon
@Disabled
@Autonomous(name = "ForwardsTest")
public class ForwardsTest extends CommandOpMode {
    public static double dist = 96;
    public static double vm = 40;
    public static double ai = 40;
    public static double af = 40;
    private MecanumDrive drive;
    @Override
    public void initOpMode() {
        drive = new MecanumDrive(this, true);
        drive.setPose(new Pose(0, 0, 0));
        scheduler.register(drive);
        Command traj = new TrajCommandBuilder(drive, new Pose(0, 0, 0))
                .setMoveConstraints(new AsymConstraints(vm, ai, af))
                .lineTo(new Vec(dist, 0))
                .pause(1)
                .lineTo(new Vec(0, 0))
                .pause(1)
                .build(scheduler);
        scheduler.schedule(new RepeatCommand(traj));
        scheduler.schedule(FnCommand.repeat(t -> {
            Pose p = drive.pose(t);
            Twist v = drive.vel(t);
            TrajectoryState state = drive.getTrajectory().state(t);
            telemetry.addData("Position", p.x);
            telemetry.addData("Velocity", v.dx);
            telemetry.addData("Target Position", state.pos.x);
            telemetry.addData("Target Velocity", state.vel.dx);
        }));
    }
}