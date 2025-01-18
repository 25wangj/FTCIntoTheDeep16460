package org.firstinspires.ftc.teamcode.autonomous;
import static org.firstinspires.ftc.teamcode.hardware.ValueStorage.*;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public abstract class AbstractAutonomous extends CommandOpMode {
    protected Robot robot;
    protected Side side = Side.BLUE;
    public abstract void initAutonomous();
    @Override
    public void initOpMode() {
        robot = new Robot(this, true);
        initAutonomous();
    }
    @Override
    public void endOpMode(double t) {
        lastSide = side;
    }
}
