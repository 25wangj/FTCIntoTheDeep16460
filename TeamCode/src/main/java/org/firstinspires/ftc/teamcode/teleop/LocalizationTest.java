package org.firstinspires.ftc.teamcode.teleop;
import static java.lang.Math.*;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.Vec;
@Photon
@Disabled
@TeleOp(name = "LocalizationTest")
public class LocalizationTest extends CommandOpMode { @Override
    public void initOpMode() {
        MecanumDrive drive = new MecanumDrive(this, true);
        drive.setPose(new Pose(0, 0, 0));
        scheduler.register(drive);
        scheduler.schedule(FnCommand.repeat(t -> {
            Vec v = new Vec(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
            double turn = -gamepad1.right_stick_x;
            if (v.norm() + abs(turn) < 0.05) {
                drive.setPowers(new Vec(0, 0), 0);
            } else {
                drive.setPowers(v, turn);
            }
            Pose p = drive.pose(t);
            telemetry.addData("X", p.x);
            telemetry.addData("Y", p.y);
            telemetry.addData("Heading", p.h % (2 * PI));
        }, drive));
    }
}
