package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import org.ejml.simple.SimpleMatrix;
public class Twist {
    public static final double EPS = 1e-6;
    public double dx;
    public double dy;
    public double dh;
    public Twist(double dx, double dy, double dh) {
        this.dx = dx;
        this.dy = dy;
        this.dh = dh;
    }
    public Twist(Vec dv, double dh) {
        this(dv.x, dv.y, dh);
    }
    public Twist(SimpleMatrix m) {
        this(m.get(0), m.get(1), m.get(2));
    }
    public Vec vec() {
        return new Vec(dx, dy);
    }
    public Twist mult(double a) {
        return new Twist(a * dx, a * dy, a * dh);
    }
    public Twist rotate(double a) {
        return new Twist(vec().rotate(a), dh);
    }
    public Pose exp() {
        SimpleMatrix exp = new SimpleMatrix(3, 3);
        exp.set(2, 2, 1);
        if (abs(dh) < EPS) {
            exp.set(0, 0, 1 - dh * dh / 6);
            exp.set(0, 1, -dh / 2);
            exp.set(1, 0, dh / 2);
            exp.set(1, 1, 1 - dh * dh / 6);
        } else {
            exp.set(0, 0, sin(dh) / dh);
            exp.set(0, 1, (cos(dh) - 1) / dh);
            exp.set(1, 0, (1 - cos(dh)) / dh);
            exp.set(1, 1, sin(dh) / dh);
        }
        return new Pose(exp.mult(new SimpleMatrix(new double[] {dx, dy, dh})));
    }
}
