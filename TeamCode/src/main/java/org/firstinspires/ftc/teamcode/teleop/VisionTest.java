package org.firstinspires.ftc.teamcode.teleop;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.RobotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.ValueStorage.*;
import static org.firstinspires.ftc.teamcode.hardware.Vision.Priority.*;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.RisingEdgeDetector;
import org.firstinspires.ftc.teamcode.hardware.Robot;
@Photon
@TeleOp(name = "VisionTest")
public class VisionTest extends CommandOpMode {
    private Robot robot;
    private Side side = null;
    @Override
    public void initOpMode() {
        sleep(250);
        telemetry.addLine("Press A for red, B for blue");
        telemetry.update();
        while (side == null && !isStopRequested()) {
            if (gamepad1.a) {
                side = Side.RED;
            } else if (gamepad1.b) {
                side = Side.BLUE;
            }
        }
        robot = new Robot(this, false, side);
        scheduler.addListener(RisingEdgeDetector.listen(() -> gamepad1.right_bumper, t -> {
            if (robot.stateMachine.transition(GRABBED, CAMERA)) {
            } else if (robot.stateMachine.transition(CAMERA, EXTEND, YELLOW_COLOR)) {
            } else if (robot.stateMachine.transition(EXTEND, GRABBED)) {}
        }));
    }
}