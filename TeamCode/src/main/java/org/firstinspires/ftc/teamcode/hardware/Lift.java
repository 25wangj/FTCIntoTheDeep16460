package org.firstinspires.ftc.teamcode.hardware;
import static java.lang.Math.*;
import static com.qualcomm.robotcore.util.Range.*;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.ParCommand;
import org.firstinspires.ftc.teamcode.command.RepeatCommand;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.command.WaitCommand;
import org.firstinspires.ftc.teamcode.control.AsymProfile;
import org.firstinspires.ftc.teamcode.control.AsymProfile.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.ChainProfile;
import org.firstinspires.ftc.teamcode.control.DelayProfile;
import org.firstinspires.ftc.teamcode.control.MotionProfile;
import org.firstinspires.ftc.teamcode.control.MotionState;
import org.firstinspires.ftc.teamcode.control.PidfCoefficients;
import org.firstinspires.ftc.teamcode.control.PidfController;
import org.firstinspires.ftc.teamcode.control.RampProfile;
import org.firstinspires.ftc.teamcode.movement.Vec;
@Config
public class Lift implements Subsystem {
    public static class LiftPosition {
        public static final double liftInToTicks = 62.9;
        public static final double turretRadToTicks = 168.4;
        public static final double pivotRadToTicks = 478.6;
        public static final double pivotRadToLiftIn = 0.32;
        public static final double armLen = 5.91;
        public final double liftExt;
        public final double turretAng;
        public final double pivotAng;
        public LiftPosition(double liftExt, double turretAng, double pivotAng) {
            this.liftExt = liftExt;
            this.turretAng = turretAng;
            this.pivotAng = pivotAng;
        }
        public static LiftPosition inverse(Vec v) {
            double turretAng = asin(v.y / armLen);
            double liftExt = v.x + armLen * (1 - cos(turretAng));
            return new LiftPosition(liftExt, turretAng, 0);
        }
        public Vec forwards() {
            return new Vec(liftExt + armLen * (cos(turretAng) - 1), armLen * sin(turretAng));
        }
        public static LiftPosition fromPos(double pivotTicks, double liftRTicks, double liftLTicks) {
            double pivotAng = pivotTicks / pivotRadToTicks;
            double turretAng = (liftRTicks - liftLTicks) / turretRadToTicks;
            double liftExt = (liftRTicks + liftLTicks) / liftInToTicks + pivotAng * pivotRadToLiftIn;
            return new LiftPosition(liftExt, turretAng, pivotAng);
        }
    }
    public static final double liftToDrive = -1.66;
    public static final double liftXMin = 0;
    public static final double liftXMax = 24;
    public static final double liftYMax = 5.5;
    public static final double pivotUp = 1.71;
    public static final LiftPosition liftHighBucket = new LiftPosition(30.5, 0, pivotUp);
    public static final LiftPosition liftLowBucket = new LiftPosition (15, 0, pivotUp);
    public static final LiftPosition liftWall1 = new LiftPosition(3.5, -PI/2, PI/2);
    public static final LiftPosition liftWall2 = new LiftPosition(3.5, -1.31, PI/2);
    public static final LiftPosition liftChamber1 = new LiftPosition(18, 0, 0.81);
    public static final LiftPosition climb1 = new LiftPosition(17.5, 0, pivotUp);
    public static final LiftPosition climb2 = new LiftPosition(16, 0, pivotUp);
    public static final LiftPosition climb3 = new LiftPosition(8, 0, pivotUp);
    public static final LiftPosition climb4 = new LiftPosition(9.2, 0, 1.22);
    public static final LiftPosition climb5 = new LiftPosition(27, 0, 1);
    public static final LiftPosition climb6 = new LiftPosition(27, 0, 1.22);
    public static final LiftPosition climb7 = new LiftPosition(25.2, 0, 1.22);
    public static final LiftPosition climb8 = new LiftPosition(21, 0, 1.22);
    public static final LiftPosition climb9 = new LiftPosition(7.5, 0, pivotUp);
    public static final LiftPosition climb10 = new LiftPosition(9.2, 0, pivotUp);
    public static final LiftPosition climb11 = new LiftPosition(9.2, 0, 1);
    public static double pivotKp = 5;
    public static double pivotKi = 0;
    public static double pivotKd = 0;
    public static double pivotKgs = 0.02;
    public static double pivotKgd = 0.013;
    public static double pivotKv = 0.15;
    public static double pivotKa = 0.01;
    public static final PidfCoefficients pivotCoeffs = new PidfCoefficients(
        pivotKp, pivotKi, pivotKd, a -> {
            MotionState pivotState = (MotionState)a[0];
            MotionState liftState = (MotionState)a[1];
            return (pivotState.x == pivotUp) ? 0.25 : (pivotKgs + pivotKgd * liftState.x) * cos(pivotState.x)
                    + pivotKv * pivotState.v + pivotKa * pivotState.a;});
    public static final PidfCoefficients pivotClimbCoeffs = new PidfCoefficients(15, 0 ,0, a -> {
            MotionState pivotState = (MotionState)a[0];
            return 0.5 * pivotState.v;});
    public static double liftKp = 0.5;
    public static double liftKi = 0.5;
    public static double liftKd = 0;
    public static double liftKgs = 0.04;
    public static double liftKgd = 0.001;
    public static double liftKs = 0.1;
    public static double liftKv = 0.012;
    public static double liftKa = 0.001;
    public static final PidfCoefficients liftCoeffs = new PidfCoefficients(
        liftKp, liftKi, liftKd, a -> {
            MotionState pivotState = (MotionState)a[0];
            MotionState liftState = (MotionState)a[1];
            return (liftKgs + liftKgd * liftState.x) * sin(pivotState.x) +
                    liftKs * signum(liftState.v) + liftKv * liftState.v + liftKa * liftState.a;});
    public static final PidfCoefficients liftClimbCoeffs = new PidfCoefficients(0.5, 0.5, 0, a -> {
       MotionState liftState = (MotionState)a[1];
       return -0.25 + 0.02 * liftState.v;});
    public static double turretKp = 2;
    public static double turretKi = 1;
    public static double turretKd = 0;
    public static double turretKs = 0.1;
    public static double turretKv = 0.02;
    public static double turretKa = 0.0005;
    public static final PidfCoefficients turretCoeffs = new PidfCoefficients(
        turretKp, turretKi, turretKd, a -> {
            MotionState turretState = (MotionState)a[0];
            return turretKs * signum(turretState.v) + turretKv * turretState.v + turretKa * turretState.a;});
    public static final PidfCoefficients turretClimbCoeffs = new PidfCoefficients(0.25, 0, 0);
    public static double pivotVm = 7;
    public static double pivotAi = 50;
    public static double pivotAf = 15;
    public static final AsymConstraints pivotDefaultConstraints = new AsymConstraints(pivotVm, pivotAi, pivotAf);
    public static final AsymConstraints pivotBackConstraints = new AsymConstraints(2, 16, 16);
    public static final AsymConstraints pivotClimbConstraints = new AsymConstraints(2, 8, 8);
    public static double liftVm = 75;
    public static double liftAi = 750;
    public static double liftAf = 200;
    public static final AsymConstraints liftDefaultConstraints = new AsymConstraints(liftVm, liftAi, liftAf);
    public static final AsymConstraints liftClimbConstraints = new AsymConstraints(20, 40, 40);
    public static double turretVm = 10;
    public static double turretAi = 50;
    public static double turretAf = 25;
    public static final AsymConstraints turretConstraints = new AsymConstraints(turretVm, turretAi, turretAf);
    private double adjustV = 40;
    private DcMotorEx pivot;
    private DcMotorEx liftR;
    private DcMotorEx liftL;
    private MecanumDrive drive;
    private PidfController pivotPidf = new PidfController(pivotCoeffs);
    private PidfController liftPidf = new PidfController(liftCoeffs);
    private PidfController turretPidf = new PidfController(turretCoeffs);
    private AsymConstraints pivotConstraints = pivotDefaultConstraints;
    private AsymConstraints liftConstraints = liftDefaultConstraints;
    private MotionProfile pivotProfile;
    private MotionProfile liftProfile;
    private MotionProfile turretProfile;
    private static double pivotOffset = 0;
    private static double liftROffset = 0;
    private static double liftLOffset = 0;
    private double zeroTime;
    private boolean climbing = false;
    public Lift(MecanumDrive drive, CommandOpMode opMode, boolean auto) {
        opMode.register(this);
        pivot = opMode.hardwareMap.get(DcMotorEx.class, "pivot");
        liftR = opMode.hardwareMap.get(DcMotorEx.class, "liftR");
        liftL = opMode.hardwareMap.get(DcMotorEx.class, "liftL");
        liftL.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.drive = drive;
        if (auto) {
            reset();
        }
        zeroTime = 0;
        pivotProfile = new DelayProfile(0, new MotionState(0), 0);
        liftProfile = new DelayProfile(0, new MotionState(0), 0);
        turretProfile = new DelayProfile(0, new MotionState(0), 0);
        pivot.setPower(-0.15);
        liftR.setPower(-0.25);
        liftL.setPower(-0.25);
    }
    public LiftPosition liftPos(double t) {
        return new LiftPosition(liftProfile.state(t).x, turretProfile.state(t).x, pivotProfile.state(t).x);
    }
    public double restTime() {
        return max(pivotProfile.tf(), max(liftProfile.tf(), turretProfile.tf()));
    }
    public void reset() {
        pivotOffset = pivot.getCurrentPosition();
        liftROffset = liftR.getCurrentPosition();
        liftLOffset = liftL.getCurrentPosition();
    }
    public Command goTo(LiftPosition pos) {
        return new FnCommand(t -> {
            if (!Double.isNaN(zeroTime)) {
                zeroTime = Double.NaN;
                reset();
            }
            if (pivotProfile.state(t).x == pos.pivotAng) {
                liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints,
                        t, new MotionState(pos.liftExt));
                turretProfile = AsymProfile.extendAsym(turretProfile, turretConstraints,
                        liftProfile.tf(), new MotionState(pos.turretAng));
            } else {
                pivotProfile = AsymProfile.extendAsym(pivotProfile, pivotConstraints,
                        t, new MotionState(pos.pivotAng));
                liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints,
                        t + 0.15, new MotionState(pos.liftExt));
                turretProfile = AsymProfile.extendAsym(turretProfile, turretConstraints,
                        liftProfile.tf(), new MotionState(pos.turretAng));
            }
        }, t -> {}, (t, b) -> {}, t -> t > restTime(), this);
    }
    public Command wallTo(LiftPosition pos) {
        return new FnCommand(t -> {
            pivotProfile = AsymProfile.extendAsym(pivotProfile, pivotConstraints,
                    t, new MotionState(pos.pivotAng));
            turretProfile = AsymProfile.extendAsym(turretProfile, turretConstraints,
                    t, new MotionState(pos.turretAng));
            liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints,
                    turretProfile.tf(), new MotionState(pos.liftExt));
        }, t -> {}, (t, b) -> {}, t -> t > restTime(), this);
    }
    public Command toWall() {
        return new FnCommand(t -> {
            liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints,
                    t, new MotionState(liftWall1.liftExt));
            turretProfile = AsymProfile.extendAsym(turretProfile, turretConstraints,
                    liftProfile.tf(), new MotionState(liftWall1.turretAng));
            double dt = AsymProfile.extendAsym(pivotProfile, pivotConstraints, t,
                    new MotionState(0)).tf() - t;
            pivotProfile = AsymProfile.extendAsym(pivotProfile, pivotConstraints,
                    max(t, turretProfile.tf() - dt), new MotionState(liftWall1.pivotAng));
        }, t -> {}, (t, b) -> {}, t -> t > restTime(), this);
    }
    public Command goBack(boolean bucket, boolean reset) {
        return new FnCommand(t -> {
            turretProfile = AsymProfile.extendAsym(turretProfile, turretConstraints,
                    t, new MotionState(0));
            if (pivotProfile.state(t).x == 0) {
                liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints,
                        turretProfile.tf(), new MotionState(0));
            } else if (bucket) {
                MotionProfile pivotBackProfile = AsymProfile.extendAsym(pivotProfile,
                        pivotBackConstraints, t, new MotionState(PI/2));
                liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints,
                        max(turretProfile.tf(), pivotBackProfile.tf()), new MotionState(0));
                double dt = AsymProfile.extendAsym(pivotBackProfile, pivotConstraints, t,
                        new MotionState(0)).tf() - t;
                pivotProfile = new ChainProfile(pivotBackProfile, AsymProfile.extendAsym(pivotBackProfile, pivotConstraints,
                        max(pivotBackProfile.tf(), liftProfile.tf() - dt + 0.15), new MotionState(0)));
            } else {
                liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints, turretProfile.tf(), new MotionState(0));
                double dt = AsymProfile.extendAsym(pivotProfile, pivotConstraints, t,
                        new MotionState(0)).tf() - t;
                pivotProfile = AsymProfile.extendAsym(pivotProfile, pivotConstraints,
                        max(t, liftProfile.tf() - dt + 0.15), new MotionState(0));
            }
        }, t -> {}, (t, b) -> {
            if (reset) {
                zeroTime = restTime();
            }}, t -> t > restTime(), this);
    }
    public Command specimen() {
        return new FnCommand(t -> {
            pivotPidf.setCoeffs(new PidfCoefficients(50, 0, 0, a -> -0.5));
            pivotProfile = AsymProfile.extendAsym(pivotProfile, pivotConstraints,
                    t, new MotionState(pivotProfile.state(t).x - 0.15));
        }, t -> {}, (t, b) -> pivotPidf.setCoeffs(pivotCoeffs), t -> t > restTime(), this);
    }
    public Command adjust(Vec v, double dt) {
        return FnCommand.once(t -> {
            LiftPosition pos = liftPos(t);
            Vec p = pos.forwards();
            double xn = clip(p.x + v.x * adjustV * dt, liftXMin, liftXMax);
            double yn = clip(p.y + v.y * adjustV * dt, -liftYMax, liftYMax);
            LiftPosition posN = LiftPosition.inverse(new Vec(xn, yn));
            liftProfile = RampProfile.extendRamp(liftProfile, t, new MotionState(posN.liftExt), dt);
            turretProfile = RampProfile.extendRamp(turretProfile, t, new MotionState(posN.turretAng), dt);
        }, true, this);
    }
    public void setClimb(boolean climbing) {
        this.climbing = climbing;
        liftPidf.reset();
        turretPidf.reset();
        pivotPidf.setCoeffs(pivotClimbCoeffs);
        pivotConstraints = pivotClimbConstraints;
        if (climbing) {
            liftPidf.setCoeffs(liftClimbCoeffs);
            turretPidf.setCoeffs(turretClimbCoeffs);
            liftConstraints = liftClimbConstraints;
        } else {
            liftPidf.setCoeffs(liftCoeffs);
            turretPidf.setCoeffs(turretCoeffs);
            liftConstraints = liftDefaultConstraints;
        }
    }
    public Command climb() {
        return new SeqCommand(
                FnCommand.once(t -> drive.setPto(true), drive),
                goTo(climb2),
                FnCommand.once(t -> setClimb(true)),
                new ParCommand(
                    goTo(climb3),
                    new WaitCommand(0.25, t -> {
                        pivotProfile = AsymProfile.extendAsym(pivotProfile,
                                pivotConstraints, t, new MotionState(1.22));
                        liftConstraints = new AsymConstraints(10, 20, 20);})),
                new WaitCommand(0.25),
                goTo(climb4),
                FnCommand.once(t -> {
                    setClimb(false);
                    drive.setPto(false);
                    drive.setPowers(new Vec(-0.5, 0), 0);}),
                new ParCommand(
                        goTo(climb5),
                        new WaitCommand(0.25, t -> drive.setPowers(new Vec(0, 0), 0))),
                goTo(climb6),
                FnCommand.once(t -> drive.setPto(true)),
                goTo(climb7),
                FnCommand.once(t -> setClimb(true)),
                goTo(climb8),
                goTo(climb9),
                new WaitCommand(0.25, t -> {
                    setClimb(false);
                    drive.setPto(false);
                    drive.setPowers(new Vec(0, 0), 0);}),
                goTo(climb10),
                FnCommand.once(t -> liftConstraints = new AsymConstraints(10, 20, 20)),
                goTo(climb11),
                FnCommand.repeat(t -> {}));
    }
    @Override
    public void update(double t, boolean active) {
        if (active) {
            LiftPosition pos = LiftPosition.fromPos(pivot.getCurrentPosition() - pivotOffset,
                    liftR.getCurrentPosition() - liftROffset, liftL.getCurrentPosition() - liftLOffset);
            MotionState pivotState = pivotProfile.state(t);
            MotionState liftState = liftProfile.state(t);
            MotionState turretState = turretProfile.state(t);
            if (pivotState.x == 0) {
                pivot.setPower(-0.15);
                pivotPidf.reset();
            } else {
                pivotPidf.set(pivotState.x);
                pivotPidf.update(t, pos.pivotAng, pivotState, liftState);
                pivot.setPower(pivotPidf.get());
            }
            if (!Double.isNaN(zeroTime)) {
                double power = (t - zeroTime < 0.25) ? -0.75 : -0.25;
                liftR.setPower(power);
                liftL.setPower(power);
                liftPidf.reset();
                turretPidf.reset();
            } else {
                liftPidf.set(liftState.x);
                turretPidf.set(turretState.x);
                liftPidf.update(t, pos.liftExt, pivotState, liftState);
                turretPidf.update(t, pos.turretAng, turretState);
                liftR.setPower(liftPidf.get() + turretPidf.get());
                liftL.setPower(liftPidf.get() - turretPidf.get());
                if (climbing) {
                    drive.setPowers(new Vec(liftPidf.get() * liftToDrive, 0), turretPidf.get() * liftToDrive);
                }
            }
        }
    }
}