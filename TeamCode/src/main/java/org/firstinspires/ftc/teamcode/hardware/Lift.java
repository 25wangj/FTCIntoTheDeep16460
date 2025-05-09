package org.firstinspires.ftc.teamcode.hardware;
import static java.lang.Double.NaN;
import static java.lang.Math.*;
import static com.qualcomm.robotcore.util.Range.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.MovementType.*;
import static org.firstinspires.ftc.teamcode.hardware.MecanumDrive.PtoState.DOWN;
import static org.firstinspires.ftc.teamcode.hardware.MecanumDrive.PtoState.PRESS;
import static org.firstinspires.ftc.teamcode.hardware.MecanumDrive.PtoState.UP;
import static org.firstinspires.ftc.teamcode.hardware.ValueStorage.*;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.ParCommand;
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
import org.firstinspires.ftc.teamcode.movement.CachingMotor;
import org.firstinspires.ftc.teamcode.movement.Vec;
@Config
public class Lift implements Subsystem {
    public enum MovementType {
        PIVOT_FIRST, PIVOT_RETRACT, LIFT_FIRST, LIFT_RETRACT, TURRET_FIRST, TURRET_LAST
    }
    public static class LiftPosition {
        public static final double liftInToTicks = 53.9;
        public static final double turretRadToTicks = 144.3;
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
    public static final double liftXMin = 5;
    public static final double liftXMax = 24;
    public static final double liftYMax = 5.5;
    public static final double pivotUp = 1.71;
    public static final LiftPosition liftHighBucket = new LiftPosition(31.5, 0, pivotUp);
    public static final LiftPosition liftLowBucket = new LiftPosition(15, 0, pivotUp);
    public static final LiftPosition liftWall1 = new LiftPosition(3.5, -PI/2, PI/2);
    public static final LiftPosition liftWall2 = new LiftPosition(3.5, -1.31, PI/2);
    public static final LiftPosition liftChamber = new LiftPosition(18, 0, 0.80);
    public static final LiftPosition climb1 = new LiftPosition(17.75, 0, pivotUp);
    public static final LiftPosition climb2 = new LiftPosition(16, 0, pivotUp);
    public static final LiftPosition climb3 = new LiftPosition(8, 0, pivotUp);
    public static final LiftPosition climb4 = new LiftPosition(9.2, 0, 1.25);
    public static final LiftPosition climb5 = new LiftPosition(27, 0, 1);
    public static final LiftPosition climb6 = new LiftPosition(27.25, 0, 1.25);
    public static final LiftPosition climb7 = new LiftPosition(25.2, 0, 1.25);
    public static final LiftPosition climb8 = new LiftPosition(7.5, 0, 1.25);
    public static final LiftPosition climb9 = new LiftPosition(9.2, 0, pivotUp);
    public static final LiftPosition climb10 = new LiftPosition(9.2, 0, 1);
    public static double pivotKp = 5;
    public static double pivotKi = 0;
    public static double pivotKd = 0;
    public static double pivotKl = 0.04;
    public static double pivotKgs = 0.02;
    public static double pivotKgd = 0.65;
    public static double pivotKv = 0.15;
    public static double pivotKa = 0.01;
    public static final PidfCoefficients pivotCoeffs = new PidfCoefficients(a -> {
            MotionState pivotState = (MotionState)a[0];
            MotionState liftState = (MotionState)a[1];
            double fl = 1 + pivotKl * liftState.x;
            double fg = 1 + pivotKgd * liftState.x;
            return new double[] {pivotKp * fl, pivotKi * fl, pivotKd * fl, (pivotState.x == pivotUp) ? liftState.x * 0.015 :
                    ((pivotKv * pivotState.v + pivotKa * pivotState.a) * fl + pivotKgs * cos(pivotState.x) * fg)};});
    public static final PidfCoefficients pivotClimbCoeffs = new PidfCoefficients(15, 0 ,0, a -> {
            MotionState pivotState = (MotionState)a[0];
            return 0.5 * pivotState.v;});
    public static double liftKp = 0.5;
    public static double liftKi = 0;
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
            if (liftState.x == 0) {
                return -0.25;
            } else if (liftState.x == liftHighBucket.liftExt) {
                return 0.4;
            }
            return (liftKgs + liftKgd * liftState.x) * sin(pivotState.x) +
                    liftKs * signum(liftState.v) + liftKv * liftState.v + liftKa * liftState.a;});
    public static final PidfCoefficients liftClimbCoeffs = new PidfCoefficients(0.8, 0.8, 0, a -> {
       MotionState liftState = (MotionState)a[1];
       return -0.4 + 0.03 * liftState.v;});
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
    public static final AsymConstraints pivotClimbConstraints = new AsymConstraints(2, 8, 8);
    public static final AsymConstraints pivotCameraConstraints = new AsymConstraints(2, 12, 12);
    public static double liftVm = 75;
    public static double liftAi = 750;
    public static double liftAf = 250;
    public static final AsymConstraints liftDefaultConstraints = new AsymConstraints(liftVm, liftAi, liftAf);
    public static final AsymConstraints liftClimbConstraints = new AsymConstraints(20, 40, 40);
    public static double turretVm = 10;
    public static double turretAi = 50;
    public static double turretAf = 25;
    public static final AsymConstraints turretConstraints = new AsymConstraints(turretVm, turretAi, turretAf);
    private double adjustV = 40;
    private CachingMotor pivot;
    private CachingMotor liftR;
    private CachingMotor liftL;
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
    public Lift(Robot robot, CommandOpMode opMode, boolean auto) {
        opMode.register(this);
        pivot = new CachingMotor(opMode.hardwareMap.get(DcMotorEx.class, "pivot"), () -> voltage);
        liftR = new CachingMotor(opMode.hardwareMap.get(DcMotorEx.class, "liftR"), () -> voltage);
        liftL = new CachingMotor(opMode.hardwareMap.get(DcMotorEx.class, "liftL"), () -> voltage);
        liftL.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftR.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftL.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.drive = robot.drive;
        if (auto) {
            zeroTime = NaN;
            reset();
            pivotProfile = new AsymProfile(pivotConstraints, System.nanoTime()*1e-9, new MotionState(0), new MotionState(liftChamber.pivotAng));
            liftProfile = new DelayProfile(0, new MotionState(0), 0);
            turretProfile = new DelayProfile(0, new MotionState(0), 0);
        } else {
            LiftPosition pos = LiftPosition.fromPos(pivot.motor.getCurrentPosition() - pivotOffset,
                    liftR.motor.getCurrentPosition() - liftROffset,
                    liftL.motor.getCurrentPosition() - liftLOffset);
            zeroTime = NaN;
            pivotProfile = new DelayProfile(0, new MotionState(pos.pivotAng), 0);
            liftProfile = new DelayProfile(0, new MotionState(pos.liftExt), 0);
            turretProfile = new DelayProfile(0, new MotionState(pos.turretAng), 0);
            opMode.schedule(goBack());
        }
    }
    public LiftPosition liftPos(double t) {
        return new LiftPosition(liftProfile.state(t).x, turretProfile.state(t).x, pivotProfile.state(t).x);
    }
    public double restTime() {
        return max(pivotProfile.tf(), max(liftProfile.tf(), turretProfile.tf()));
    }
    public void reset() {
        pivotOffset = pivot.motor.getCurrentPosition();
        liftROffset = liftR.motor.getCurrentPosition();
        liftLOffset = liftL.motor.getCurrentPosition();
    }
    public Command pivotTo(double pivotAng, AsymConstraints... pivotOverride) {
        return new FnCommand(t -> {
            pivotProfile = AsymProfile.extendAsym(pivotProfile, pivotOverride.length > 0 ? pivotOverride[0] : pivotConstraints,
                    t, new MotionState(pivotAng));
        }, t -> {}, (t, b) -> {}, t -> t > pivotProfile.tf(), this);
    }
    public Command liftTo(double liftExt, AsymConstraints... liftOverride) {
        return new FnCommand(t -> {
            liftProfile = AsymProfile.extendAsym(liftProfile, liftOverride.length > 0 ? liftOverride[0] : liftConstraints,
                    t, new MotionState(liftExt));
        }, t -> {}, (t, b) -> {}, t -> t > liftProfile.tf(), this);
    }
    public Command turretTo(double turretAng, AsymConstraints... turretOverride) {
        return new FnCommand(t -> {
            turretProfile = AsymProfile.extendAsym(turretProfile, turretOverride.length > 0 ? turretOverride[0] : turretConstraints, 
                    t, new MotionState(turretAng));
        }, t -> {}, (t, b) -> {}, t -> t > turretProfile.tf(), this);
    }
    public Command goTo(LiftPosition pos, MovementType type, AsymConstraints... pivotOverride) {
        return new FnCommand(t -> {
            if (!Double.isNaN(zeroTime)) {
                zeroTime = NaN;
                reset();
            }
            if (pivotOverride.length > 0) {
                pivotConstraints = pivotOverride[0];
            }
            switch (type) {
                case PIVOT_FIRST:
                    pivotProfile = AsymProfile.extendAsym(pivotProfile, pivotConstraints,
                            t, new MotionState(pos.pivotAng));
                    double dt1 = AsymProfile.extendAsym(liftProfile, liftConstraints,
                            t, new MotionState(pos.liftExt)).tf() - t;
                    double liftTi1 = max(pivotProfile.tf() - min(dt1, 0.35), t);
                    liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints,
                            liftTi1, new MotionState(pos.liftExt));
                    turretProfile = AsymProfile.extendAsym(turretProfile, turretConstraints,
                            liftProfile.tf(), new MotionState(pos.turretAng));
                    break;
                case PIVOT_RETRACT:
                    turretProfile = AsymProfile.extendAsym(turretProfile, turretConstraints,
                            t, new MotionState(0));
                    liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints, turretProfile.tf(),
                            new MotionState(min(min(liftProfile.state(t).x, pos.liftExt), 3)));
                    double dt2 = AsymProfile.extendAsym(pivotProfile, pivotConstraints,
                            t, new MotionState(pos.pivotAng)).tf() - t;
                    double dt3 = new AsymProfile(liftConstraints, 0, new MotionState(0),
                            new MotionState(pos.liftExt)).tf();
                    double pivotTi1 = max(t, liftProfile.tf() - max(dt2 - min(dt3, 0.35), 0));
                    double liftTi2 = max(pivotTi1, pivotTi1 + dt2 - min(dt3, 0.35));
                    pivotProfile = AsymProfile.extendAsym(pivotProfile, pivotConstraints,
                            pivotTi1, new MotionState(pos.pivotAng));
                    liftProfile = new ChainProfile(liftProfile, AsymProfile.extendAsym(liftProfile,
                            liftConstraints, liftTi2, new MotionState(pos.liftExt)));
                    turretProfile = new ChainProfile(turretProfile, AsymProfile.extendAsym(turretProfile,
                            turretConstraints, liftProfile.tf(), new MotionState(pos.turretAng)));
                    break;
                case LIFT_FIRST:
                    turretProfile = AsymProfile.extendAsym(turretProfile, turretConstraints,
                            t, new MotionState(pos.turretAng));
                    liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints,
                            turretProfile.tf(), new MotionState(pos.liftExt));
                    double dt4 = AsymProfile.extendAsym(pivotProfile, pivotConstraints,
                            t, new MotionState(pos.pivotAng)).tf() - t;
                    double pivotTi2 = max(liftProfile.tf() - min(dt4, 0.35), t);
                    pivotProfile = AsymProfile.extendAsym(pivotProfile, pivotConstraints,
                            pivotTi2, new MotionState(pos.pivotAng));
                    break;
                case LIFT_RETRACT:
                    turretProfile = AsymProfile.extendAsym(turretProfile, turretConstraints,
                            t, new MotionState(0));
                    liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints, turretProfile.tf(),
                        new MotionState(min(min(liftProfile.state(t).x, pos.liftExt), 3)));
                    double dt5 = AsymProfile.extendAsym(pivotProfile, pivotConstraints,
                            t, new MotionState(pos.pivotAng)).tf() - t;
                    double pivotTi3 = max(liftProfile.tf() - min(dt5, 0.35), t);
                    pivotProfile = AsymProfile.extendAsym(pivotProfile, pivotConstraints,
                            pivotTi3, new MotionState(pos.pivotAng));
                    double dt6 = new AsymProfile(liftConstraints, 0, new MotionState(0),
                            new MotionState(pos.liftExt)).tf();
                    double liftTi3 = max(liftProfile.tf(), pivotProfile.tf() - dt6);
                    liftProfile = new ChainProfile(liftProfile, AsymProfile.extendAsym(liftProfile,
                            liftConstraints, liftTi3, new MotionState(pos.liftExt)));
                    turretProfile = new ChainProfile(turretProfile, AsymProfile.extendAsym(turretProfile,
                            turretConstraints, liftProfile.tf(), new MotionState(pos.turretAng)));
                    break;
                case TURRET_FIRST:
                    pivotProfile = AsymProfile.extendAsym(pivotProfile, pivotConstraints,
                            t, new MotionState(pos.pivotAng));
                    turretProfile = AsymProfile.extendAsym(turretProfile, turretConstraints,
                            t, new MotionState(pos.turretAng));
                    double liftTi4 = max(turretProfile.tf(), t + 0.15);
                    liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints,
                            liftTi4, new MotionState(pos.liftExt));
                    break;
                case TURRET_LAST:
                    liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints,
                            t, new MotionState(pos.liftExt));
                    turretProfile = AsymProfile.extendAsym(turretProfile, turretConstraints,
                            liftProfile.tf(), new MotionState(pos.turretAng));
                    double dt7 = AsymProfile.extendAsym(pivotProfile, pivotConstraints, t,
                            new MotionState(0)).tf() - t;
                    double pivotTi4 = max(t, turretProfile.tf() - dt7);
                    pivotProfile = AsymProfile.extendAsym(pivotProfile, pivotConstraints,
                            pivotTi4, new MotionState(pos.pivotAng));
                    break;
            }
            if (pivotOverride.length > 0) {
                pivotConstraints = pivotDefaultConstraints;
            }
        }, t -> {}, (t, b) -> {}, t -> t > restTime(), this);
    }
    public Command goBack() {
        return new SeqCommand(
                goTo(new LiftPosition(0, 0, 0), MovementType.LIFT_FIRST),
                FnCommand.once(t -> zeroTime = t));
    }
    public Command specimen(boolean push) {
        return new FnCommand(t -> {
            pivotPidf.setCoeffs(new PidfCoefficients(50, 0, 0, a -> -0.75));
            liftPidf.setCoeffs(new PidfCoefficients(0.5, 0, 0, a -> 0.5));
            if (push) {
                turretPidf.setCoeffs(new PidfCoefficients(10, 1, 0, a -> 0.5));
            }
            pivotProfile = AsymProfile.extendAsym(pivotProfile, pivotConstraints,
                    t, new MotionState(pivotProfile.state(t).x - 0.1));
        }, t -> {}, (t, b) -> {
            pivotPidf.setCoeffs(pivotCoeffs);
            liftPidf.setCoeffs(liftCoeffs);
            turretPidf.setCoeffs(turretCoeffs);
        }, t -> t > restTime() + 0.1, this);
    }
    public Command adjust(Vec v, double dt) {
        return FnCommand.once(t -> {
            LiftPosition pos = liftPos(t);
            Vec p = pos.forwards();
            double xn = clip(p.x + v.x * adjustV * dt, 0, liftXMax);
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
                FnCommand.once(t -> drive.setPto(DOWN), drive),
                goTo(climb2, PIVOT_FIRST),
                FnCommand.once(t -> setClimb(true)),
                new ParCommand(
                    goTo(climb3, PIVOT_FIRST),
                    new WaitCommand(0.25, t ->
                        pivotProfile = AsymProfile.extendAsym(pivotProfile,
                                pivotConstraints, t, new MotionState(1.25)))),
                FnCommand.once(t -> {
                    liftConstraints = new AsymConstraints(10, 20, 20);
                    drive.setPto(PRESS);}),
                goTo(climb4, PIVOT_FIRST),
                FnCommand.once(t -> {
                    setClimb(false);
                    drive.setPto(UP);
                    drive.setPowers(new Vec(-0.5, 0), 0);}),
                new ParCommand(
                        goTo(climb5, PIVOT_FIRST),
                        new WaitCommand(0.25, t -> drive.setPowers(new Vec(0, 0), 0))),
                goTo(climb6, PIVOT_FIRST),
                FnCommand.once(t -> drive.setPto(DOWN)),
                goTo(climb7, PIVOT_FIRST),
                FnCommand.once(t -> setClimb(true)),
                new ParCommand(
                        goTo(climb8, PIVOT_FIRST),
                        new WaitCommand(0.5, t ->
                            pivotProfile = AsymProfile.extendAsym(pivotProfile,
                                    pivotConstraints, t, new MotionState(pivotUp)))),
                new WaitCommand(0.25, t -> {
                    liftConstraints = new AsymConstraints(10, 20, 20);
                    drive.setPto(PRESS);}),
                goTo(climb9, PIVOT_FIRST),
                FnCommand.once(t -> {
                    setClimb(false);
                    drive.setPto(UP);
                    drive.setPowers(new Vec(0, 0), 0);}),
                goTo(climb10, PIVOT_FIRST),
                FnCommand.repeat(t -> {}));
    }
    @Override
    public void update(double t, boolean active) {
        LiftPosition pos = LiftPosition.fromPos(pivot.motor.getCurrentPosition() - pivotOffset,
                liftR.motor.getCurrentPosition() - liftROffset,
                liftL.motor.getCurrentPosition() - liftLOffset);
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
            if (!climbing) {
                liftR.setPower(liftPidf.get() + turretPidf.get());
                liftL.setPower(liftPidf.get() - turretPidf.get());
            } else {
                liftR.setPower(0);
                liftL.setPower(0);
                drive.setPowers(new Vec(-liftPidf.get(), 0), -turretPidf.get());
            }
        }
    }
}