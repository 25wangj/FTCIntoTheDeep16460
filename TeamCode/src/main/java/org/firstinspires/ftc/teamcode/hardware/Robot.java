package org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.StateMachine;
public class Robot {
    public final MecanumDrive drive;
    public final Lift lift;
    public final Arm arm;
    public StateMachine<RobotStateMachine.robotStates> stateMachine;
    public Robot(CommandOpMode opMode, boolean auto) {
        drive = new MecanumDrive(opMode, auto);
        lift = new Lift(drive, opMode, auto);
        arm = new Arm(opMode, auto);
        stateMachine = RobotStateMachine.get(opMode, this, RobotStateMachine.robotStates.GRABBED);
    }
}