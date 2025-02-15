package org.firstinspires.ftc.teamcode.hardware;
import static org.firstinspires.ftc.teamcode.hardware.ValueStorage.*;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.StateMachine;
import org.firstinspires.ftc.teamcode.vision.Vision;
public class Robot {
    public final MecanumDrive drive;
    public final Lift lift;
    public final Arm arm;
    //public final Vision vision;
    public StateMachine<RobotStateMachine.robotStates> stateMachine;
    public Robot(CommandOpMode opMode, boolean auto, Side side, double time) {
        drive = new MecanumDrive(opMode, auto, time);
        lift = new Lift(this, opMode, auto, time);
        arm = new Arm(opMode, auto, time);
        //vision = new Vision(this, opMode, side);
        //vision.start();
        stateMachine = RobotStateMachine.get(opMode, this, RobotStateMachine.robotStates.GRABBED);
    }
}