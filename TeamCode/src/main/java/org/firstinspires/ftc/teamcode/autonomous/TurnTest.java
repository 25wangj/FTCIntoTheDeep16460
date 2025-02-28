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

@Config
@Photon
@Disabled
@Autonomous(name = "TurnTest")
public class TurnTest extends CommandOpMode {
    public static double ang = PI;
    public static double vm = 6;
    public static double ai = 12;
    public static double af = 12;
    private MecanumDrive drive;
    @Override
    public void initOpMode() {
        drive = new MecanumDrive(this, true);
        drive.setPose(new Pose(0, 0, 0));
        scheduler.register(drive);
        Command traj = new TrajCommandBuilder(drive, new Pose(0, 0, 0))
                .setTurnConstraints(new AsymConstraints(vm, ai, af))
                .turn(ang)
                .pause(1)
                .turn(0)
                .pause(1)
                .build(scheduler);
        scheduler.schedule(new RepeatCommand(traj));
        scheduler.schedule(FnCommand.repeat(t -> {
            Pose p = drive.pose(t);
            Twist v = drive.vel(t);
            TrajectoryState state = drive.getTrajectory().state(t);
            telemetry.addData("Angle", p.h);
            telemetry.addData("Angular Velocity", v.dh);
            telemetry.addData("Target Angle", state.pos.h);
            telemetry.addData("Target Velocity", state.vel.dh);
        }));
    }
}