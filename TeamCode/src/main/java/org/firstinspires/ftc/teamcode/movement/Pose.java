package org.firstinspires.ftc.teamcode.movement;
import org.ejml.simple.SimpleMatrix;
public class Pose {
    public final double x;
    public final double y;
    public final double h;
    public Pose(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }
    public Pose(Vec v, double h) {
        this(v.x, v.y, h);
    }
    public Pose(SimpleMatrix m) {
        this(m.get(0), m.get(1), m.get(2));
    }
    public Vec vec() {
        return new Vec(x, y);
    }
    public Pose add(Pose other) {
        return new Pose(vec().combo(1, other.vec().rotate(h), 1), h + other.h);
    }
}
