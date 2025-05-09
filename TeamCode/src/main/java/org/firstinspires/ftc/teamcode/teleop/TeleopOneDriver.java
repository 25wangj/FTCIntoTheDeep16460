package org.firstinspires.ftc.teamcode.teleop;
import static org.firstinspires.ftc.teamcode.hardware.Lift.MovementType.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.RobotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.ValueStorage.*;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import static org.firstinspires.ftc.teamcode.movement.CachingMotor.refVoltage;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.ParCommand;
import org.firstinspires.ftc.teamcode.command.RisingEdgeDetector;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.Vec;
@Photon
@TeleOp(name = "OneDriver")
public class TeleopOneDriver extends CommandOpMode {
    private Robot robot;
    private double grabRot = 0;
    @Override
    public void initOpMode() {
        robot = new Robot(this, false, lastSide);
        scheduler.addListener(RisingEdgeDetector.listen(() -> gamepad1.ps, robot.drive.setHeading(-PI/2)),
            RisingEdgeDetector.listen(() -> gamepad1.a, t -> {
                if (robot.stateMachine.transition(GRABBED, BUCKET, liftHighBucket)) {
                } else if (robot.stateMachine.transition(BUCKET, BUCKET, liftHighBucket)) {
                } else if (robot.stateMachine.state() == EXTEND || robot.stateMachine.state() == EXTEND_GRAB) {
                    schedule(robot.lift.goTo(LiftPosition.inverse(new Vec(14, 0)), PIVOT_FIRST));
                }}),
            RisingEdgeDetector.listen(() -> gamepad1.b, t -> {
                if (robot.stateMachine.transition(GRABBED, BUCKET, liftLowBucket)) {
                } else if (robot.stateMachine.transition(BUCKET, BUCKET, liftLowBucket)) {
                } else if (robot.stateMachine.state() == EXTEND || robot.stateMachine.state() == EXTEND_GRAB) {
                    schedule(robot.lift.goTo(LiftPosition.inverse(new Vec(22, 0)), PIVOT_FIRST));
                }}),
            RisingEdgeDetector.listen(() -> gamepad1.x, t -> {
                if (robot.stateMachine.transition(GRABBED, WALL)) {
                } else if (robot.stateMachine.state() == EXTEND || robot.stateMachine.state() == EXTEND_GRAB) {
                    schedule(robot.lift.goTo(LiftPosition.inverse(new Vec(6, 0)), PIVOT_FIRST));
                } else if (!scheduler.using(robot.drive) && robot.stateMachine.transition(WALL, CHAMBER)) {}}),
            RisingEdgeDetector.listen(() -> gamepad1.right_bumper, t -> {
                if (robot.stateMachine.transition(EXTEND, EXTEND_GRAB)) {
                } else if (robot.stateMachine.transition(EXTEND_GRAB, GRABBED)) {
                } else if (robot.stateMachine.transition(BUCKET, EXTEND,
                        LiftPosition.inverse(new Vec(0, 0)), grabRot - robot.drive.pose(t).h)) {
                } else if (robot.stateMachine.transition(WALL, WALL, true)) {
                } else if (!scheduler.using(robot.drive) && robot.stateMachine.transition(CHAMBER, WALL, false)) {}}),
            RisingEdgeDetector.listen(() -> gamepad1.left_bumper, t -> {
                if (robot.stateMachine.transition(EXTEND_GRAB, EXTEND)) {
                } else if (robot.stateMachine.transition(GRABBED, EXTEND,
                        LiftPosition.inverse(new Vec(12, 0)), grabRot - robot.drive.pose(t).h)) {
                } else if (robot.stateMachine.transition(WALL, EXTEND,
                        LiftPosition.inverse(new Vec(0, 0)), grabRot - robot.drive.pose(t).h, PIVOT_RETRACT, true, true)) {
                } else if (!scheduler.using(robot.drive) && robot.stateMachine.transition(CHAMBER, EXTEND,
                        LiftPosition.inverse(new Vec(0, 0)), grabRot - robot.drive.pose(t).h, PIVOT_RETRACT, false)) {}}),
            RisingEdgeDetector.listen(() -> gamepad1.start, t -> {
                if (robot.stateMachine.transition(GRABBED, CLIMB)) {
                } else if (robot.stateMachine.transition(CLIMB, CLIMBED)) {}}),
            RisingEdgeDetector.listen(() -> gamepad1.back, t -> {
                if (robot.stateMachine.transition(CLIMB, GRABBED)) {}}));
        schedule(FnCommand.repeat(t -> {
            if (robot.stateMachine.state() == EXTEND || robot.stateMachine.state() == EXTEND_GRAB) {
                schedule(robot.arm.setGrab(grabRot - robot.drive.pose(t).h, robot.lift));
            }
            if (gamepad1.left_trigger > 0.1) {
                Vec ang = new Vec(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
                if (ang.norm() > 0.5) {
                    grabRot = ang.angle() - PI/2;
                }
                if (robot.stateMachine.state() == EXTEND || robot.stateMachine.state() == EXTEND_GRAB) {
                    Vec pos = new Vec(-gamepad1.right_stick_y, /*-gamepad1.right_stick_x*/0);
                    if (pos.norm() > 0.05) {
                        schedule(robot.lift.adjust(pos/*.rotate(-robot.drive.getHeading())*/.mult(pos.norm()), 0.05));
                    }
                }
            } else if (!scheduler.using(robot.drive)) {
                double f = max(voltage / refVoltage, 1);
                f *= gamepad1.right_trigger > 0.1 ? 0.25 : 1;
                Vec p = new Vec(-gamepad1.left_stick_y * f, -gamepad1.left_stick_x * f)
                        .rotate(-robot.drive.pose(t).h - PI/2);
                double turn = -gamepad1.right_stick_x * f;
                if (p.norm() + abs(turn) < 0.05) {
                    robot.drive.setPowers(new Vec(0, 0), 0);
                } else {
                    robot.drive.setPowers(p, turn);
                }
            }
        }));
    }
}