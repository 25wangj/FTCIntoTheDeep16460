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
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.Vec;
@Photon
@TeleOp(name = "TwoDriver")
public class TeleopTwoDriver extends CommandOpMode {
    private Robot robot;
    private double grabRot = 0;
    @Override
    public void initOpMode() {
        robot = new Robot(this, false);
        scheduler.addListener(RisingEdgeDetector.listen(() -> gamepad1.ps, robot.drive.setHeading(0)),
                RisingEdgeDetector.listen(() -> gamepad2.a, t -> {
                    if (robot.stateMachine.transition(GRABBED, BUCKET, liftHighBucket)) {
                    } else if (robot.stateMachine.transition(GRABBED, BUCKET, liftHighBucket)) {
                    } else if (robot.stateMachine.state() == EXTEND || robot.stateMachine.state() == EXTEND_GRAB) {
                        schedule(robot.lift.goTo(LiftPosition.inverse(new Vec(14, 0))));
                    }}),
                RisingEdgeDetector.listen(() -> gamepad2.b, t -> {
                    if (robot.stateMachine.transition(GRABBED, BUCKET, liftLowBucket)) {
                    } else if (robot.stateMachine.transition(GRABBED, BUCKET, liftLowBucket)) {
                    } else if (robot.stateMachine.state() == EXTEND || robot.stateMachine.state() == EXTEND_GRAB) {
                        schedule(robot.lift.goTo(LiftPosition.inverse(new Vec(22, 0))));
                    }}),
                RisingEdgeDetector.listen(() -> gamepad2.x, t -> {
                    if (robot.stateMachine.transition(GRABBED, WALL)) {
                    } else if (robot.stateMachine.state() == EXTEND || robot.stateMachine.state() == EXTEND_GRAB) {
                        schedule(robot.lift.goTo(LiftPosition.inverse(new Vec(6, 0))));
                    }}),
                RisingEdgeDetector.listen(() -> gamepad2.right_bumper, t -> {
                    if (robot.stateMachine.transition(EXTEND, EXTEND_GRAB)) {
                    } else if (robot.stateMachine.transition(EXTEND_GRAB, GRABBED)) {
                    } else if (robot.stateMachine.transition(BUCKET, EXTEND,
                            new Pose(0, 0, grabRot - robot.drive.getHeading(t)))) {
                    } else if (robot.stateMachine.transition(WALL, WALL, 0d)) {}}),
                RisingEdgeDetector.listen(() -> gamepad2.left_bumper, t -> {
                    if (robot.stateMachine.transition(EXTEND_GRAB, EXTEND)) {
                    } else if (robot.stateMachine.transition(GRABBED, EXTEND,
                            new Pose(12, 0, grabRot - robot.drive.getHeading(t)))) {
                    } else if (robot.stateMachine.transition(WALL, GRABBED)) {}}),
                RisingEdgeDetector.listen(() -> gamepad1.start, t -> {
                    if (robot.stateMachine.transition(GRABBED, CLIMB)) {
                    } else if (robot.stateMachine.transition(CLIMB, CLIMBED)) {}}),
                RisingEdgeDetector.listen(() -> gamepad1.back, t -> {
                    if (robot.stateMachine.transition(CLIMB, GRABBED)) {}}));
        schedule(robot.drive.setHeading(lastPose.h + (lastSide == Side.BLUE ? 1 : -1) * PI/2));
        schedule(FnCommand.repeat(t -> {
            if (robot.stateMachine.state() == EXTEND || robot.stateMachine.state() == EXTEND_GRAB) {
                schedule(robot.arm.setGrab(grabRot - robot.drive.getHeading(t), robot.lift));
                Vec pos = new Vec(-gamepad2.right_stick_y, /*-gamepad2.right_stick_x*/0);
                if (pos.norm() > 0.05) {
                    schedule(robot.lift.adjust(pos/*.rotate(-robot.drive.getHeading())*/.mult(pos.norm()), 0.05));
                }
            }
            Vec ang = new Vec(-gamepad2.left_stick_y, -gamepad2.left_stick_x);
            if (ang.norm() > 0.5) {
                grabRot = ang.angle();
            }
            if (!scheduler.using(robot.drive)) {
                double f = gamepad1.right_trigger > 0.1 ? 0.25 : 1;
                Vec p = new Vec(-gamepad1.left_stick_y * f, -gamepad1.left_stick_x * f)
                        .rotate(-robot.drive.getHeading(t));
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