package org.firstinspires.ftc.teamcode.hardware;
import static org.firstinspires.ftc.teamcode.hardware.ValueStorage.*;

import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.StateMachine;
import org.firstinspires.ftc.teamcode.movement.Pose;

public class Robot {
    public final MecanumDrive drive;
    public final Lift lift;
    public final Arm arm;
    public final Vision vision;
    public StateMachine<RobotStateMachine.RobotStates> stateMachine;
    public Robot(CommandOpMode opMode, boolean auto, Side side) {
        voltage = opMode.hardwareMap.voltageSensor.iterator().next().getVoltage();
        drive = new MecanumDrive(opMode, auto);
        lift = new Lift(this, opMode, auto);
        arm = new Arm(opMode, auto);
        vision = new Vision(this, opMode, side);
        stateMachine = RobotStateMachine.get(opMode, this, RobotStateMachine.RobotStates.GRABBED);
    }
}