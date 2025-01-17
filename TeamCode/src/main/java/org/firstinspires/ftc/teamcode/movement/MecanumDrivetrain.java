package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Math.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.control.AsymProfile.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.PidfCoefficients;
import org.firstinspires.ftc.teamcode.control.PidfController;
public abstract class MecanumDrivetrain extends Drivetrain {
    protected DcMotorEx fr;
    protected DcMotorEx fl;
    protected DcMotorEx br;
    protected DcMotorEx bl;
    private double trackWidth;
    private double ks;
    private double kv;
    private double ka;
    private double strafeMult;
    private PidfController xPid;
    private PidfController yPid;
    private PidfController turnPid;
    public MecanumDrivetrain(double trackWidth, double ks, double kv, double ka, double strafeMult, PidfCoefficients xCoeffs, PidfCoefficients yCoeffs, PidfCoefficients turnCoeffs, AsymConstraints moveConstraints, AsymConstraints turnConstraints) {
        super(moveConstraints, turnConstraints);
        this.trackWidth = trackWidth;
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
        this.strafeMult = strafeMult;
        xPid = new PidfController(xCoeffs);
        yPid = new PidfController(yCoeffs);
        turnPid = new PidfController(turnCoeffs);
    }
    public void setPowers(Vec v, double t) {
        double d = max(abs(v.x) + abs(strafeMult * v.y) + abs(t), 1);
        fr.setPower(offset((v.x + strafeMult * v.y + t) / d, ks));
        fl.setPower(offset((v.x - strafeMult * v.y - t) / d, ks));
        br.setPower(offset((v.x - strafeMult * v.y + t) / d, ks));
        bl.setPower(offset((v.x + strafeMult * v.y - t) / d, ks));
    }
    @Override
    public void follow(double time) {
        TrajectoryState state = traj.state(time);
        Pose acPos = pose(time);
        Twist acVel = vel(time);
        Vec locPosError = acPos.vec().combo(1, state.pos.vec(), -1).rotate(-acPos.h);
        Vec locVelError = acVel.vec().combo(1, state.vel.vec(), -1).rotate(-acPos.h);
        Vec locVel = state.vel.vec().rotate(-acPos.h);
        Vec locAccel = state.accel.rotate(-acPos.h);
        xPid.derivUpdate(time, locPosError.x, locVelError.x, null);
        yPid.derivUpdate(time, locPosError.y, locVelError.y, null);
        turnPid.derivUpdate(time, ((acPos.h - state.pos.h) % (2 * PI) + 3 * PI) % (2 * PI) - PI, acVel.dh - state.vel.dh, null);
        setPowers(new Vec(xPid.get(), yPid.get()).combo(1, locVel, kv).combo(1, locAccel, ka),
                turnPid.get() + kv * state.vel.dh * trackWidth);
    }
    private double offset(double a, double b) {
        return a + signum(a) * b;
    }
}
