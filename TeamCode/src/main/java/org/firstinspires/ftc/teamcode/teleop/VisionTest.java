package org.firstinspires.ftc.teamcode.teleop;
import static org.firstinspires.ftc.teamcode.hardware.Arm.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.MovementType.PIVOT_FIRST;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.RobotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.ValueStorage.*;
import static org.firstinspires.ftc.teamcode.hardware.Vision.*;
import static org.firstinspires.ftc.teamcode.vision.VisionValueStorage.*;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.RisingEdgeDetector;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@Photon
@TeleOp(name = "VisionTest")
public class VisionTest extends CommandOpMode {
    private Robot robot;
    private Side side = null;
    private Boolean sample = null;
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
        telemetry.addLine("Press X for sample, Y for specimen");
        telemetry.update();
        while (sample == null && !isStopRequested()) {
            if (gamepad1.x) {
                sample = true;
            } else if (gamepad1.y) {
                sample = false;
            }
        }
        telemetry.update();
        robot = new Robot(this, false, side);
        robot.vision.sampleProc.telemetry = telemetry;
        takeFrame = true;
        if (sample) {
            vals = bucketVals;
            schedule(robot.lift.goTo(visionBucket.liftPos, PIVOT_FIRST));
        } else {
            vals = chamberVals;
            schedule(robot.lift.goTo(visionChamber.liftPos, PIVOT_FIRST));
        }
        schedule(FnCommand.once(t -> {
            robot.arm.setArm(armGrab(0));
            robot.arm.setClaw(false);}));
        scheduler.addListener(RisingEdgeDetector.listen(() -> gamepad1.right_bumper, t -> {
            robot.vision.portal.setProcessorEnabled(robot.vision.colorProc, false);
            robot.vision.portal.setProcessorEnabled(robot.vision.sampleProc, true);}));
        scheduler.addListener(RisingEdgeDetector.listen(() -> gamepad1.left_bumper, t -> {
            robot.vision.portal.setProcessorEnabled(robot.vision.sampleProc, false);
            robot.vision.portal.setProcessorEnabled(robot.vision.colorProc, true);}));
    }
}