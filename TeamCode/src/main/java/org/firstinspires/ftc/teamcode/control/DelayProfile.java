package org.firstinspires.ftc.teamcode.control;
public class DelayProfile extends MotionProfile {
    public DelayProfile(double ti, MotionState i, double dt) {
        this.ti = ti;
        this.i = i;
        this.tf = ti + dt;
    }
    @Override
    public MotionState state(double t) {
        return new MotionState(i.x + i.v * (t - ti), i.v);
    }
    public static DelayProfile extendDelay(MotionProfile p, double ti, double dt) {
        return new DelayProfile(ti, p.state(ti), dt);
    }
    public static DelayProfile extendDelay(MotionProfile p, double dt) {
        return extendDelay(p, p.tf, dt);
    }
}