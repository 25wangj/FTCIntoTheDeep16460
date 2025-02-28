package org.firstinspires.ftc.teamcode.hardware;
import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.hardware.ValueStorage.Side;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.vision.SampleProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
public class Vision {
    public enum Priority {
        YELLOW_ONLY, COLOR_ONLY, YELLOW_COLOR, COLOR_YELLOW
    }
    public static final double maxDelay = 1;
    public static double sampleTime;
    public static Pose samplePose = new Pose(12, 0, 0);
    private VisionPortal portal;
    private SampleProcessor proc;
    public Vision(Robot robot, CommandOpMode opMode, Side side) {
        try {
            proc = new SampleProcessor(side == Side.RED);
            WebcamName camera = opMode.hardwareMap.get(WebcamName.class, "camera");
            if (camera == null) {
                throw new RuntimeException();
            }
            portal = new VisionPortal.Builder()
                    .setCamera(camera)
                    .setCameraResolution(new Size(320, 240))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .addProcessor(proc)
                    .build();
            opMode.telemetry.addLine("Starting Camera");
            opMode.telemetry.update();
            while (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                if (portal.getCameraState() == VisionPortal.CameraState.ERROR) {
                    throw new RuntimeException();
                }
            }
            opMode.telemetry.addLine("Camera Started");
            opMode.telemetry.update();
            portal.setProcessorEnabled(proc, true);
        } catch (RuntimeException e) {
            opMode.telemetry.addLine("Failed to Start Camera");
            opMode.telemetry.update();
        }
    }
    public static Pose samplePose() {
        return samplePose;
    }
}