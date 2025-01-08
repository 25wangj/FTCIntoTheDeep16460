package org.firstinspires.ftc.teamcode.hardware;
import static java.lang.Math.*;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.Subsystem;
public class Arm implements Subsystem {
    public static class ArmPosition {
        public static final double armZero = 0.59;
        public static final double armRange = 2.59;
        public static final double diffZero = 0.24;
        public static final double diffRange = 6.11;
        public static final double diffRatio = 0.833;
        public final double armAng;
        public final double wristAng;
        public final double wristRot;
        public final double armPos;
        public final double diffRPos;
        public final double diffLPos;
        public ArmPosition(double armAng, double wristAng, double wristRot) {
            this.armAng = armAng;
            this.wristAng = wristAng;
            this.wristRot = wristRot;
            armPos = armAng / armRange + armZero;
            diffRPos = ((wristAng + wristRot) * diffRatio + armAng) / diffRange + diffZero;
            diffLPos = ((wristAng - wristRot) * diffRatio + armAng) / diffRange + diffZero;
        }
    }
    public static final double armUp = 0.35;
    public static final ArmPosition armRest(double rot) {return new ArmPosition(0, 0, rot);}
    public static final ArmPosition armGrab(double rot) {return new ArmPosition(armUp,0, rot);}
    public static final ArmPosition armBucket = new ArmPosition(armUp, 2.65, -PI/2);
    public static final ArmPosition armSideChamber1 = new ArmPosition(-0.35, 3.32, -1.83);
    public static final ArmPosition armSideChamber2 = new ArmPosition(armUp, 3.32, -1.83);
    public static final ArmPosition armBackChamber = new ArmPosition(armUp, 2.79, 0);
    public static final ArmPosition armGrabbed = new ArmPosition(armUp, 1.89, -PI/2);
    public static final double clawOpen = 0.06;
    public static final double clawClosed = 0.6;
    public static final double grabHyst = 0.262;
    private ServoImplEx arm;
    private ServoImplEx diffR;
    private ServoImplEx diffL;
    private ServoImplEx claw;
    private ArmPosition armPos = null;
    private boolean wristFlipped = false;
    private boolean outside = true;
    public Arm(CommandOpMode opMode, boolean auto) {
        opMode.register(this);
        arm = opMode.hardwareMap.get(ServoImplEx.class, "arm");
        diffR = opMode.hardwareMap.get(ServoImplEx.class, "diffR");
        diffL = opMode.hardwareMap.get(ServoImplEx.class, "diffL");
        claw = opMode.hardwareMap.get(ServoImplEx.class, "claw");
        arm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        diffR.setPwmRange(new PwmControl.PwmRange(500, 2500));
        diffL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));
        if (auto) {
            armPos = armGrabbed;
            opMode.schedule(FnCommand.once(t -> setClaw(true), this));
        } else {
            opMode.schedule(FnCommand.once(t -> {
                armPos = armGrabbed;
                setClaw(false);}, this));
        }
    }
    public void setArm(ArmPosition pos) {
        armPos = pos;
    }
    public void setClaw(boolean closed) {
        claw.setPosition(closed ? clawClosed : clawOpen);
    }
    public ArmPosition armPos() {
        return armPos;
    }
    public Command setGrab(double angle, Lift lift) {
        return FnCommand.once(t -> {
            double relAng = angle - lift.liftPos(lift.restTime()).turretAng;
            double rot = (relAng % (2 * PI) + 3 * PI) % (2 * PI) - PI;
            double flippedRot = ((relAng + PI) % (2 * PI) + 3 * PI) % (2 * PI) - PI;
            if ((!wristFlipped && abs(rot) < PI / 2 + grabHyst) ||
                    (wristFlipped && abs(flippedRot) > PI / 2 + grabHyst)) {
                wristFlipped = false;
                armPos = new ArmPosition(armPos.armAng, armPos.wristAng, rot);
            } else {
                wristFlipped = true;
                armPos = new ArmPosition(armPos.armAng, armPos.wristAng, flippedRot);
            }
        }, true, this);
    }
    public void update(double t, boolean active) {
        if (armPos != null) {
            arm.setPosition(armPos.armPos);
            diffR.setPosition(armPos.diffRPos);
            diffL.setPosition(armPos.diffLPos);
        }
    }
}
