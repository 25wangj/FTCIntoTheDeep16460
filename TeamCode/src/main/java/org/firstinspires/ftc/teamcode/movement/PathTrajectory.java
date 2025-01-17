package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Double.isNaN;
import static java.lang.Math.*;
import org.firstinspires.ftc.teamcode.control.AsymProfile.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.AsymProfile;
import org.firstinspires.ftc.teamcode.control.MotionState;
public class PathTrajectory implements Trajectory {
    private Path[] paths;
    private AsymProfile moveProfile;
    private AsymProfile[] turnProfiles;
    private double[] lens;
    private double len;
    private double[] hs;
    private double[] tfs;
    private double ti;
    public PathTrajectory(Path[] paths, double[] hs, AsymConstraints moveConstraints,
                          AsymConstraints turnConstraints, double vi, double vf) {
        this.paths = paths;
        lens = new double[paths.length + 1];
        this.hs = hs;
        turnProfiles = new AsymProfile[paths.length + 1];
        for (int i = 0; i < paths.length; i++) {
            lens[i + 1] = lens[i] + paths[i].length();
            if (isNaN(hs[i + 1])) {
                hs[i + 1] = hs[i] + paths[i].state(1).dir.angle() - paths[i].state(0).dir.angle();
            } else {
                turnProfiles[i] = new AsymProfile(turnConstraints, 0,
                        new MotionState(hs[i]), new MotionState(hs[i + 1]));
            }
        }
        len = lens[paths.length];
        moveProfile = new AsymProfile(moveConstraints, 0,
                new MotionState(0, vi), new MotionState(len, vf));
        double[] fs = new double[paths.length];
        double max = 1;
        for (int i = 0; i < paths.length; i++) {
            if (turnProfiles[i] != null) {
                fs[i] = turnProfiles[i].tf() / (t(lens[i + 1]) - t(lens[i]));
                max = max(max, fs[i]);
            }
        }
        moveProfile = new AsymProfile(moveConstraints.scaleT(1 / max), 0,
                new MotionState(0, vi), new MotionState(len, vf));
        tfs = new double[paths.length + 1];
        for (int i = 0; i < paths.length; i++) {
            tfs[i + 1] = t(lens[i + 1]);
            if (fs[i] != 0) {
                turnProfiles[i] = new AsymProfile(turnConstraints.scaleT(fs[i] / max), tfs[i],
                        new MotionState(hs[i]), new MotionState(hs[i + 1]));
            }
        }
    }
    @Override
    public TrajectoryState state(double t) {
        MotionState s = moveProfile.state(t - ti);
        int ind = 0;
        for (int i = 0; i < paths.length; i++) {
            if (t - ti < tfs[i + 1]) {
                ind = i;
                break;
            }
        }
        PathState state = paths[ind].state((s.x - lens[ind]) / (lens[ind + 1] - lens[ind]));
        MotionState h;
        if (turnProfiles[ind] == null) {
            h = new MotionState(hs[ind] + state.dir.angle() - paths[ind].state(0).dir.angle(), state.curv*s.v);
        } else {
            h = turnProfiles[ind].state(t - ti);
        }
        return new TrajectoryState(new Pose(state.pos, h.x), new Twist(state.dir.mult(s.v), h.v),
                state.dir.combo(moveProfile.state(t - ti).a, new Vec(state.dir.y, -state.dir.x), state.curv*s.v*s.v));
    }
    private double t(double x) {
        if (x < moveProfile.state(moveProfile.t1()).x) {
            return sqrt(2 * x / moveProfile.c().ai);
        } else if (x < moveProfile.state(moveProfile.t2()).x) {
            return moveProfile.t1() + (x - moveProfile.state(moveProfile.t1()).x) / moveProfile.c().vm;
        } else {
            return moveProfile.tf() - sqrt(2 * (moveProfile.state(moveProfile.tf()).x - x) / moveProfile.c().af);
        }
    }
    @Override
    public void setTi(double ti) {
        this.ti = ti;
    }
    @Override
    public double tf() {
        return ti + tfs[paths.length];
    }
    public double[] tfs() {
        return tfs;
    }
}