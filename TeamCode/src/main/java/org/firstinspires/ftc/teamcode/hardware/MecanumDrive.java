package org.firstinspires.ftc.teamcode.hardware;
import static org.firstinspires.ftc.teamcode.hardware.ValueStorage.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.RepeatCommand;
import org.firstinspires.ftc.teamcode.command.WaitCommand;
import org.firstinspires.ftc.teamcode.control.AsymProfile.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.PidfCoefficients;
import org.firstinspires.ftc.teamcode.movement.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.movement.Pose;
@Config
public class MecanumDrive extends MecanumDrivetrain {
    public static double trackWidth = 10.4;
    public static double driveKv = 0.011;
    public static double driveKa = 0.001;
    public static double driveKs = 0;
    public static double strafeMult = 1;
    public static final double otosDelay = 0.01;
    public static final Pose otosOffset = new Pose(2.25, 0, 3.13);
    public static final double linScalar = 1.012;
    public static final double angScalar = 0.996;
    public static double xKp = 0.25;
    public static double xKi = 0;
    public static double xKd = 0;
    public static double yKp = 0.25;
    public static double yKi = 0;
    public static double yKd = 0;
    public static double turnKp = 2;
    public static double turnKi = 0;
    public static double turnKd = 0;
    public static final AsymConstraints moveConstraints = new AsymConstraints(70, 70, 50);
    public static final AsymConstraints turnConstraints = new AsymConstraints(6, 12, 12);
    public static final double ptoRDown = 0.44;
    public static final double ptoRUp = 0.50;
    public static final double ptoLDown = 0.58;
    public static final double ptoLUp = 0.52;
    private Servo ptoR;
    private Servo ptoL;
    private Otos otos;
    private double headingOffset = 0;
    public MecanumDrive(CommandOpMode opMode, boolean auto) {
        super(trackWidth, driveKs, driveKv, driveKa, strafeMult, new PidfCoefficients(xKp, xKi, xKd),
                new PidfCoefficients(yKp, yKi, yKd), new PidfCoefficients(turnKp, turnKi, turnKd), moveConstraints, turnConstraints);
        opMode.register(this);
        fr = opMode.hardwareMap.get(DcMotorEx.class, "fr");
        fl = opMode.hardwareMap.get(DcMotorEx.class, "fl");
        br = opMode.hardwareMap.get(DcMotorEx.class, "br");
        bl = opMode.hardwareMap.get(DcMotorEx.class, "bl");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ptoR = opMode.hardwareMap.get(Servo.class, "ptoR");
        ptoL = opMode.hardwareMap.get(Servo.class, "ptoL");
        otos = opMode.hardwareMap.get(Otos.class, "otos");
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);
        otos.setOffset(otosOffset);
        otos.setLinearScalar(linScalar);
        otos.setAngularScalar(angScalar);
        if (auto) {
            telemetry.addLine("Calibrating");
            telemetry.update();
            otos.calibrateImu(255, true);
            telemetry.addLine("Calibrated");
            telemetry.update();
        }
        localizer = new OtosLocalizer(otos, otosDelay);
        setPto(false);
    }
    public void setPto(boolean down) {
        ptoR.setPosition(down ? ptoRDown : ptoRUp);
        ptoL.setPosition(down ? ptoLDown : ptoLUp);
    }
    public Command setHeading(double h) {
        return FnCommand.once(t -> localizer.setPose(new Pose(pose(t).vec(), h)));
    }
}