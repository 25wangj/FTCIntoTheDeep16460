package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Math.*;
import org.ejml.simple.SimpleMatrix;
public class ThreeWheelLocalizer implements Localizer {
    private final double EPS = 1e-6;
    private Pose pos = new Pose(0, 0, 0);
    private Twist vel = new Twist(0, 0, 0);
    private Encoder enc1;
    private Encoder enc2;
    private Encoder enc3;
    private double lastTime = Double.NaN;
    private double last1;
    private double last2;
    private double last3;
    private double f;
    private SimpleMatrix kin;
    public ThreeWheelLocalizer(Encoder enc1, Encoder enc2, Encoder enc3, Pose p1, Pose p2, Pose p3, double f) {
        this.enc1 = enc1;
        this.enc2 = enc2;
        this.enc3 = enc3;
        this.f = f;
        kin = new SimpleMatrix(new double[][] {
                {cos(p1.h), sin(p1.h), p1.x * sin(p1.h) - p1.y * cos(p1.h)},
                {cos(p2.h), sin(p2.h), p2.x * sin(p2.h) - p2.y * cos(p2.h)},
                {cos(p3.h), sin(p3.h), p3.x * sin(p3.h) - p3.y * cos(p3.h)}}).invert();
    }
    public ThreeWheelLocalizer(Encoder par1, Encoder par2, Encoder perp, double parDist, double perpDist, double conv) {
        this(par1, par2, perp, new Pose(0, parDist / 2, 0), new Pose(0, -parDist / 2, 0), new Pose(perpDist, 0, PI / 2), conv);
    }
    @Override
    public Pose pos(double t) {
        if (Double.isNaN(lastTime)) {
            return pos;
        }
        return pos.add(vel(t).mult(t - lastTime).exp());
    }
    @Override
    public Twist vel(double t) {
        return vel;
    }
    @Override
    public void setPose(Pose p) {
        pos = p;
        vel = new Twist(0, 0, 0);
    }
    @Override
    public void update(double time) {
        double p1 = f * enc1.getPosition();
        double p2 = f * enc2.getPosition();
        double p3 = f * enc3.getPosition();
        if (p1 == last1 && p2 == last2 && p3 == last3) {
            vel = new Twist(0, 0, 0);
        } else if (!Double.isNaN(lastTime)) {
            double dt = time - lastTime;
            Twist twist = new Twist(kin.mult(new SimpleMatrix(new double[] {p1 - last1, p2 - last2, p3 - last3})));
            vel = twist.rotate(pos.h).mult(1 / dt);
            pos = pos.add(twist.exp());
        }
        lastTime = time;
        last1 = p1;
        last2 = p2;
        last3 = p3;
    }
}
