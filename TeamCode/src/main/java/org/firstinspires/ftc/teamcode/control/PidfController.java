package org.firstinspires.ftc.teamcode.control;
public class PidfController {
    private PidfCoefficients coeffs;
    private double setPoint = 0;
    private double e = 0;
    private double i = 0;
    private double d = 0;
    private double lastTime = Double.NaN;
    private double lastE = 0;
    private double val = 0;
    public PidfController(PidfCoefficients coeffs) {
        this.coeffs = coeffs;
    }
    public void reset() {
        lastTime = Double.NaN;
        i = 0;
    }
    public void setCoeffs(PidfCoefficients coeffs) {
        this.coeffs = coeffs;
    }
    public void set(double newSetPoint) {
        setPoint = newSetPoint;
    }
    public double get() {
        return val;
    }
    public void update(double time, double pt, Object... ff) {
        e = setPoint - pt;
        if (!Double.isNaN(lastTime)) {
            double dt = time - lastTime;
            i += (e + lastE) * dt / 2;
            d = (e - lastE) / dt;
        }
        if (e * lastE < 0) {
            reset();
        }
        double[] k = coeffs.k.apply(ff);
        val = k[0] * e + k[1] * i + k[2] * d + k[3];
        lastTime = time;
        lastE = e;
    }
    public void derivUpdate(double time, double pt, double deriv, Object... ff) {
        if (!Double.isNaN(d)) {
            this.d = deriv;
        }
        e = setPoint - pt;
        if (!Double.isNaN(lastTime)) {
            i += (e + lastE) * (time - lastTime) / 2;
        }
        if (e * lastE < 0) {
            reset();
        }
        double[] k = coeffs.k.apply(ff);
        val = k[0] * e + k[1] * i + k[2] * d + k[3];
        lastTime = time;
        lastE = e;
    }
}