package org.firstinspires.ftc.teamcode.control;
import java.util.function.Function;
import java.util.function.ToDoubleFunction;
public class PidfCoefficients {
    public final Function<Object[], double[]> k;
    public PidfCoefficients(Function<Object[], double[]> k) {
        this.k = k;
    }
    public PidfCoefficients(double kp, double ki, double kd, ToDoubleFunction<Object[]> kf) {
        this(a -> new double[] {kp, ki, kd, kf.applyAsDouble(a)});
    }
    public PidfCoefficients(double kp, double ki, double kd) {
        this(kp, ki, kd, x -> 0);
    }
}