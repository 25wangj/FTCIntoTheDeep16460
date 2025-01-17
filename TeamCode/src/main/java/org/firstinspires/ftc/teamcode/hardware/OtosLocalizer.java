package org.firstinspires.ftc.teamcode.hardware;
import android.util.Pair;
import org.firstinspires.ftc.teamcode.movement.Localizer;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.Twist;
public class OtosLocalizer implements Localizer {
    private Pose pos = new Pose(0, 0, 0);
    private Twist vel = new Twist(0, 0, 0);
    private Otos otos;
    private double lastTime = Double.NaN;
    private double delay;
    public OtosLocalizer(Otos otos, double delay) {
        this.otos = otos;
        this.delay = delay;
    }
    @Override
    public Pose pos(double t) {
        if (Double.isNaN(lastTime)) {
            return pos;
        }
        Pose dp = vel(t).mult(t - lastTime).exp();
        return pos;
        //return new Pose(pos.vec().combo(1, dp.vec(), 1), pos.h + dp.h);
    }
    @Override
    public Twist vel(double t) {
        return vel;
    }
    @Override
    public void setPose(Pose p) {
        pos = p;
        vel = new Twist(0, 0, 0);
        otos.setPosition(p);
    }
    @Override
    public void update(double time) {
        if (Double.isNaN(lastTime) || time - lastTime > delay) {
            Pair<Pose, Twist> posVel = otos.getPosVel();
            pos = posVel.first;
            vel = posVel.second;
            lastTime = time;
        }
    }
}