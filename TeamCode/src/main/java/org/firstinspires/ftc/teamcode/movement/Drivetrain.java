package org.firstinspires.ftc.teamcode.movement;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.control.AsymProfile.AsymConstraints;
public abstract class Drivetrain implements Subsystem {
    private AsymConstraints moveConstraints;
    private AsymConstraints turnConstraints;
    protected Trajectory traj;
    protected Localizer localizer;
    public Drivetrain(AsymConstraints moveConstraints, AsymConstraints turnConstraints) {
        this.moveConstraints = moveConstraints;
        this.turnConstraints = turnConstraints;
    }
    public Pose pose(double t) {
        return localizer.pos(t);
    }
    public Twist vel(double t) {
        return localizer.vel(t);
    }
    public void setPose(Pose p) {
        localizer.setPose(p);
    }
    public Trajectory getTrajectory() {
        return traj;
    }
    public void setTrajectory(Trajectory traj) {
        this.traj = traj;
    }
    public AsymConstraints getMoveConstraints() {
        return moveConstraints;
    }
    public AsymConstraints getTurnConstraints() {
        return turnConstraints;
    }
    public abstract void follow(double time);
    public void updateDrivetrain(double time, boolean active) {}
    @Override
    public void update(double time, boolean active) {
        updateDrivetrain(time, active);
        if (active && localizer != null) {
            localizer.update(time);
            if (traj != null) {
                follow(time);
            }
        }
    }
}