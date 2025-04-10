package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Math.*;
public class Vec {
    public final double x;
    public final double y;
    public Vec(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public double norm() {
        return Math.sqrt(x * x + y * y);
    }
    public Vec normalize() {
        if (x == 0 && y == 0) {
            return new Vec(0, 0);
        }
        return mult(1 / norm());
    }
    public Vec mult(double a) {
        return new Vec(a * x, a * y);
    }
    public Vec rotate(double a) {
        return new Vec(x * cos(a) - y * sin(a), x * sin(a) + y * cos(a));
    }
    public Vec combo(double a, Vec other, double b) {
        return new Vec(a * x + b * other.x, a * y + b * other.y);
    }
    public double dot(Vec other) {
        return x * other.x + y * other.y;
    }
    public double angle() {
        return atan2(y, x);
    }
    public static Vec dir(double a) {
        return new Vec(cos(a), sin(a));
    }
    public boolean equals(Object o) {
        return o instanceof Vec && ((Vec)o).x == x && ((Vec)o).y == y;
    }
}