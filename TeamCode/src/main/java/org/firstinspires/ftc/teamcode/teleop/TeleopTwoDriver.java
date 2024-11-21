package org.firstinspires.ftc.teamcode.teleop;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.robotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.ValueStorage.*;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.RisingEdgeDetector;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.movement.Vec;
@Photon
@TeleOp(name = "TwoDriver")
public class TeleopTwoDriver extends CommandOpMode {
    private Robot robot;
    private double grabRot = 0;
    @Override
    public void initOpMode() {
        robot = new Robot(this, false);
        robot.drive.setHeading(lastPose.h + (lastSide == Side.BLUE ? PI / 2 : -PI / 2));
        scheduler.addListener(RisingEdgeDetector.listen(() -> gamepad1.ps, t -> robot.drive.setHeading(0)),
                RisingEdgeDetector.listen(() -> gamepad2.a, t -> {
                    if (robot.stateMachine.state() == INTAKE) {
                        robot.stateMachine.transition(EXTEND, LiftPosition.inverse(new Vec(12, 0)), grabRot);
                    } else if (robot.stateMachine.state() == GRABBED || robot.stateMachine.state() == BUCKET) {
                        robot.stateMachine.transition(BUCKET, liftHighBucket);
                    }}),
                RisingEdgeDetector.listen(() -> gamepad2.b, t -> {
                    if (robot.stateMachine.state() == GRABBED || robot.stateMachine.state() == BUCKET) {
                        robot.stateMachine.transition(BUCKET, liftLowBucket);
                    }}),
                RisingEdgeDetector.listen(() -> gamepad2.x, t -> {
                    if (robot.stateMachine.state() == GRABBED || robot.stateMachine.state() == SIDE_CHAMBER) {
                        robot.stateMachine.transition(SIDE_CHAMBER, liftHighSideChamber);
                    }}),
                RisingEdgeDetector.listen(() -> gamepad2.y, t -> {
                    if (robot.stateMachine.state() == GRABBED || robot.stateMachine.state() == SIDE_CHAMBER) {
                        robot.stateMachine.transition(SIDE_CHAMBER, liftLowSideChamber);
                    }}),
                RisingEdgeDetector.listen(() -> gamepad2.right_bumper, t -> {
                    if (robot.stateMachine.state() == INTAKE || robot.stateMachine.state() == EXTEND_GRAB) {
                        robot.stateMachine.transition(GRABBED);
                    } else if (robot.stateMachine.state() == EXTEND) {
                        robot.stateMachine.transition(EXTEND_GRAB);
                    } else if (robot.stateMachine.state() == GRABBED) {
                        robot.stateMachine.transition(SPECIMEN);
                    } else if (robot.stateMachine.state() == BUCKET  || robot.stateMachine.state() == SIDE_CHAMBER || robot.stateMachine.state() == SPECIMEN) {
                        robot.stateMachine.transition(INTAKE);
                    }}),
                RisingEdgeDetector.listen(() -> gamepad2.left_bumper, t -> {
                    if (robot.stateMachine.state() == EXTEND_GRAB) {
                        robot.stateMachine.transition(EXTEND);
                    } else if (robot.stateMachine.state() == GRABBED) {
                        robot.stateMachine.transition(INTAKE);
                    }}),
                RisingEdgeDetector.listen(() -> gamepad1.start, t -> {
                    if (robot.stateMachine.state() == INTAKE) {
                        robot.stateMachine.transition(CLIMB);
                    } else if (robot.stateMachine.state() == CLIMB) {
                        robot.stateMachine.transition(CLIMBED);
                    }}),
                RisingEdgeDetector.listen(() -> gamepad1.back, t -> {
                    if (robot.stateMachine.state() == CLIMB) {
                        robot.stateMachine.transition(INTAKE);
                    }}));
        schedule(FnCommand.repeat(t -> {
            if (robot.stateMachine.state() == EXTEND || robot.stateMachine.state() == EXTEND_GRAB) {
                robot.arm.setGrab(grabRot - robot.drive.getHeading(), robot.lift);
                Vec pos = new Vec(-gamepad2.right_stick_y, /*-gamepad2.right_stick_x*/0);
                if (pos.norm() > 0.05) {
                    schedule(robot.lift.adjust(pos/*.rotate(-robot.drive.getHeading())*/.mult(pos.norm()), 0.05));
                }
            }
            if (robot.stateMachine.state() == INTAKE || robot.stateMachine.state() == SPECIMEN) {
                if (gamepad2.dpad_down) {
                    robot.intake.set(-1);
                } else {
                    robot.intake.set(0.375);
                }
            }
            Vec ang = new Vec(-gamepad2.left_stick_y, -gamepad2.left_stick_x);
            if (ang.norm() > 0.5) {
                grabRot = ang.angle();
            }
            double f = gamepad1.right_trigger > 0.1 ? 0.25 : 1;
            Vec p = new Vec(-gamepad1.left_stick_y * f, -gamepad1.left_stick_x * f).rotate(-robot.drive.getHeading());
            double turn = -gamepad1.right_stick_x * f;
            if (p.norm() + abs(turn) < 0.05) {
                robot.drive.setPowers(new Vec(0, 0), 0);
            } else {
                robot.drive.setPowers(p, turn);
            }
        }, true, robot.drive));
    }
}