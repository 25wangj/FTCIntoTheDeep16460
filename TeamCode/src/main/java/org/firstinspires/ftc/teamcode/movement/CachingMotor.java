package org.firstinspires.ftc.teamcode.movement;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.function.DoubleSupplier;

public class CachingMotor {
    public static final int steps = 20;
    public static final int refVoltage = 12;
    public final DcMotorEx motor;
    private DoubleSupplier voltage;
    public CachingMotor(DcMotorEx motor, DoubleSupplier voltage) {
        this.motor = motor;
        this.voltage = voltage;
    }
    public void setPower(double p) {
        motor.setPower((double)Math.round(p * steps * refVoltage / voltage.getAsDouble()) / steps);
    }
}
