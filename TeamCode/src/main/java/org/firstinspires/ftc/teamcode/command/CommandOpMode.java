package org.firstinspires.ftc.teamcode.command;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ValueStorage;
public abstract class CommandOpMode extends LinearOpMode {
    public Scheduler scheduler;
    private double alpha = 0.75;
    private double speed = 0;
    private boolean done = false;
    public abstract void initOpMode();
    public void waitOpMode() {}
    public void startOpMode() {}
    public void endOpMode(double time) {}
    @Override
    public void runOpMode() {
        scheduler = new Scheduler();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ValueStorage.telemetry = telemetry;
        initOpMode();
        while (!isStarted() && !isStopRequested()) {
            waitOpMode();
            scheduler.run(false);
            telemetry.addLine("Waiting");
            telemetry.update();
        }
        startOpMode();
        while (opModeIsActive() && !done) {
            speed = alpha * speed + (1 - alpha) / scheduler.run(true);
            telemetry.addData("Loop speed", speed);
            telemetry.update();
        }
        endOpMode(scheduler.lastTime);
    }
    public void register(Subsystem... subsystems) {
        scheduler.register(subsystems);
    }
    public boolean schedule(Command command) {
        return scheduler.schedule(command);
    }
    public void end() {
        done = true;
    }
}
