package org.firstinspires.ftc.teamcode.teleop;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.teamcode.hardware.Arm.ArmPosition.armZero;
import static java.lang.Math.*;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.command.RisingEdgeDetector;
@Photon
@Disabled
@TeleOp(name = "ServoTest")
public class ServoTest extends CommandOpMode {
    private ServoImplEx servo;
    private double pos;
    @Override
    public void initOpMode() {
        servo = hardwareMap.get(ServoImplEx.class, "claw");
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        pos = 0.5;
        Subsystem servoSubsystem = (t, b) -> {
            servo.setPosition(pos);
            telemetry.addData("Position", pos);
        };
        register(servoSubsystem);
        scheduler.addListener(
            RisingEdgeDetector.listen(() -> gamepad1.a, FnCommand.once(t -> pos = min(pos + 0.1, 1))),
            RisingEdgeDetector.listen(() -> gamepad1.b, FnCommand.once(t -> pos = max(pos - 0.1, 0))),
            RisingEdgeDetector.listen(() -> gamepad1.x, FnCommand.once(t -> pos = min(pos + 0.005, 1))),
            RisingEdgeDetector.listen(() -> gamepad1.y, FnCommand.once(t -> pos = max(pos - 0.005, 0))));
    }
}