package org.firstinspires.ftc.teamcode.autonomous;
import static org.firstinspires.ftc.teamcode.hardware.ValueStorage.*;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.movement.Pose;

public abstract class AbstractAutonomous extends CommandOpMode {
    protected Robot robot;
    protected Pose start = null;
    protected Side side = null;
    public void chooseConfig() {}
    public abstract void initAutonomous();
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
        chooseConfig();
        lastSide = side;
        robot = new Robot(this, true, side);
        initAutonomous();
    }
    @Override
    public void startOpMode() {
        robot.drive.setPose(start);
    }
}
