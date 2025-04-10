package org.firstinspires.ftc.teamcode.movement;
public class WaitTrajectory implements Trajectory {
    private double ti;
    private double seconds;
    private Pose pos;
    private Vec vel;
    public WaitTrajectory(Pose pos, Vec vel) {
        this(pos, vel, Double.NaN);
    }
    public WaitTrajectory(Pose pos, Vec vel, double seconds) {
        this.seconds = seconds;
        this.pos = pos;
        this.vel = vel;
    }
    @Override
    public TrajectoryState state(double t) {
        return new TrajectoryState(new Pose(pos.vec().combo(1, vel, t - ti), pos.h),
                new Twist(vel, 0), new Vec(0, 0));
    }
    @Override
    public void setTi(double ti) {
        this.ti = ti;
    }
    @Override
    public double tf() {
        return Double.isNaN(seconds) ? Double.MAX_VALUE : ti + seconds;
    }
    @Override
    public double[] tfs() {
        return new double[] {0, seconds};
    }
}
