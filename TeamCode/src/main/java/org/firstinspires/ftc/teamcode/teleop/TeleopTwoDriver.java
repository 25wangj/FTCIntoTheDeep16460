package org.firstinspires.ftc.teamcode.teleop;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.robotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.ValueStorage.*;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.Listener;
import org.firstinspires.ftc.teamcode.command.RisingEdgeDetector;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotStateMachine;
import org.firstinspires.ftc.teamcode.movement.Vec;
@Photon
@TeleOp(name = "TwoDriver")
public class TeleopTwoDriver extends CommandOpMode {
    private Robot robot;
    private RobotStateMachine.robotStates lastState = BUCKET;
    private LiftPosition lastPos = liftHighBucket;
    private double grabRot = 0;
    @Override
    public void initOpMode() {
        robot = new Robot(this, false);
        robot.drive.setHeading(lastPose.h + (lastSide == Side.BLUE ? PI / 2 : -PI / 2));
        scheduler.addListener(RisingEdgeDetector.listen(() -> gamepad1.ps,
                        FnCommand.once(t -> robot.drive.setHeading(0))),
                RisingEdgeDetector.listen(() -> gamepad2.a, FnCommand.once(t -> {
                    if (robot.stateMachine.state() == INTAKE) {
                        robot.stateMachine.transition(EXTEND);
                    } else if (robot.stateMachine.state() == GRABBED || robot.stateMachine.state() == BUCKET) {
                        robot.stateMachine.transition(BUCKET, liftHighBucket);
                        lastState = BUCKET;
                        lastPos = liftHighBucket;
                    }})),
                RisingEdgeDetector.listen(() -> gamepad2.b, FnCommand.once(t -> {
                    if (robot.stateMachine.state() == GRABBED || robot.stateMachine.state() == BUCKET) {
                        robot.stateMachine.transition(BUCKET, liftLowBucket);
                        lastState = BUCKET;
                        lastPos = liftLowBucket;
                    }
                })),
                RisingEdgeDetector.listen(() -> gamepad2.x, FnCommand.once(t -> {
                    if (robot.stateMachine.state() == GRABBED || robot.stateMachine.state() == CHAMBER) {
                        robot.stateMachine.transition(CHAMBER, liftHighChamber);
                    }
                })),
                RisingEdgeDetector.listen(() -> gamepad2.y, FnCommand.once(t -> {
                    if (robot.stateMachine.state() == GRABBED || robot.stateMachine.state() == CHAMBER) {
                        robot.stateMachine.transition(CHAMBER, liftLowChamber);
                    }
                })),
                RisingEdgeDetector.listen(() -> gamepad2.right_bumper, FnCommand.once(t -> {
                    if (robot.stateMachine.state() == INTAKE || robot.stateMachine.state() == EXTEND_GRAB) {
                        grabRot = 0;
                        robot.stateMachine.transition(GRABBED);
                    } else if (robot.stateMachine.state() == EXTEND) {
                        robot.stateMachine.transition(EXTEND_GRAB);
                    } else if (robot.stateMachine.state() == GRABBED) {
                        robot.stateMachine.transition(SPECIMEN);
                    } else if (robot.stateMachine.state() == BUCKET  || robot.stateMachine.state() == CHAMBER || robot.stateMachine.state() == SPECIMEN) {
                        robot.stateMachine.transition(INTAKE);
                    }
                })),
                RisingEdgeDetector.listen(() -> gamepad2.left_bumper, FnCommand.once(t -> {
                    if (robot.stateMachine.state() == INTAKE || robot.stateMachine.state() == EXTEND_GRAB) {
                        robot.stateMachine.transition(EXTEND);
                    } else if (robot.stateMachine.state() == GRABBED) {
                        robot.stateMachine.transition(INTAKE);
                    }
                })),
                new Listener(() -> new Vec(gamepad2.left_stick_x, gamepad2.left_stick_y).norm() > 0.5,
                    FnCommand.once(t -> {
                        if (new Vec(gamepad2.left_stick_x, gamepad2.left_stick_y).norm() > 0.5) {
                            grabRot = atan2(-gamepad2.left_stick_x, -gamepad2.left_stick_y);
                        }})));
        schedule(FnCommand.repeat(t -> {
            if (robot.stateMachine.state() == EXTEND) {
                robot.lift.setGrab(grabRot - robot.drive.getHeading(), armUp, t);
            } else if (robot.stateMachine.state() == EXTEND_GRAB) {
                robot.lift.setGrab(grabRot - robot.drive.getHeading(), 0, t);
            }
            double f = gamepad1.right_trigger > 0.1 ? 0.25 : 1;
            Vec p = new Vec(-gamepad1.left_stick_y * f, -gamepad1.left_stick_x * f).rotate(-robot.drive.getHeading());
            double turn = -gamepad1.right_stick_x * f;
            if (p.norm() + abs(turn) < 0.05) {
                robot.drive.setPowers(new Vec(0, 0), 0);
            } else {
                robot.drive.setPowers(p, turn);
            }
        }, robot.drive));
    }
}