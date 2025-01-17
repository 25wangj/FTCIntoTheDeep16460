package org.firstinspires.ftc.teamcode.movement;
import org.firstinspires.ftc.teamcode.control.AsymProfile.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.AsymProfile;
import org.firstinspires.ftc.teamcode.control.MotionProfile;
import org.firstinspires.ftc.teamcode.control.MotionState;

public class TurnTrajectory implements Trajectory {
    private double ti;
    private Vec pos;
    private MotionProfile profile;
    public TurnTrajectory(AsymConstraints constraints, Pose pos, double h) {
        this.pos = pos.vec();
        profile = new AsymProfile(constraints, ti, new MotionState(pos.h), new MotionState(h));
    }
    @Override
    public TrajectoryState state(double t) {
        MotionState s = profile.state(t - ti);
        return new TrajectoryState(new Pose(pos, s.x), new Twist(0, 0, s.v), new Vec(0, 0));
    }
    @Override
    public void setTi(double ti) {
        this.ti = ti;
    }
    @Override
    public double tf() {
        return profile.tf() + ti;
    }
    public double[] tfs() {
        return new double[] {0, profile.tf()};
    }
}
