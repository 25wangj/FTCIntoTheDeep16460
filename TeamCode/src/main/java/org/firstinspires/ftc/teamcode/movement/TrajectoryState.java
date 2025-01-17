package org.firstinspires.ftc.teamcode.movement;
public class TrajectoryState {
    public final Pose pos;
    public final Twist vel;
    public final Vec accel;
    public TrajectoryState(Pose pos, Twist vel, Vec accel) {
        this.pos = pos;
        this.vel = vel;
        this.accel = accel;
    }
}
