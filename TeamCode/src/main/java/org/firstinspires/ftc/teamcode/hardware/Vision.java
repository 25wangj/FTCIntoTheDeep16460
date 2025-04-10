package org.firstinspires.ftc.teamcode.hardware;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.vision.VisionValueStorage.*;
import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.hardware.ValueStorage.Side;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.vision.SampleProcessor;
import org.firstinspires.ftc.teamcode.vision.ThresholdProcessor;
import org.firstinspires.ftc.teamcode.vision.VisionValueStorage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.*;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class Vision {
    public static class VisionPosition {
        public final LiftPosition liftPos;
        public final VisionValues vals;
        public VisionPosition(LiftPosition liftPos, VisionValues vals) {
            this.liftPos = liftPos;
            this.vals = vals;
        }
    }
    public static final VisionPosition visionBucket = new VisionPosition(new LiftPosition(14, 0, 0.49), bucketVals);
    public static final VisionPosition visionChamber = new VisionPosition(new LiftPosition(18, 0, 0.70), chamberVals);
    public static final double maxDelay = 0.5;
    public static final Pose dtToLift = new Pose(5.4, 0, 0);
    public double sampleTime;
    public Pose samplePose = new Pose(0, 0, 0);
    public List<Pose> allPoses = new ArrayList<>();
    public List<Pose> yellowPoses = new ArrayList<>();
    public List<Pose> colorPoses = new ArrayList<>();
    private Robot robot;
    public VisionPortal portal;
    public final SampleProcessor sampleProc;
    public final ThresholdProcessor colorProc;
    public Vision(Robot robot, CommandOpMode opMode, Side side) {
        this.robot = robot;
        sampleProc = new SampleProcessor(side == Side.RED);
        colorProc = new ThresholdProcessor();
        try {
            WebcamName camera = opMode.hardwareMap.get(WebcamName.class, "camera");
            if (camera == null) {
                throw new RuntimeException();
            }
            portal = new VisionPortal.Builder()
                    .setShowStatsOverlay(false)
                    .setCamera(camera)
                    .setCameraResolution(new Size(320, 240))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .addProcessor(sampleProc)
                    .addProcessor(colorProc)
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
            portal.setProcessorEnabled(sampleProc, true);
            portal.setProcessorEnabled(colorProc, false);
        } catch (RuntimeException e) {
            opMode.telemetry.addLine("Failed to Start Camera");
            opMode.telemetry.update();
        }
    }
    public Command takeFrame(VisionPosition pos) {
        return new FnCommand(t -> {
                VisionValueStorage.yellowPoints = new ArrayList<>();
                VisionValueStorage.colorPoints = new ArrayList<>();
                VisionValueStorage.vals = pos.vals;
                VisionValueStorage.takeFrame = true;
                sampleTime = t;
                samplePose = robot.drive.pose(sampleTime);
            }, t -> {}, (t, b) -> {
                yellowPoses = VisionValueStorage.yellowPoints.stream()
                        .map(a -> samplePose.add(new Pose(a)))
                        .collect(Collectors.toList());
                colorPoses = VisionValueStorage.colorPoints.stream()
                        .map(a -> samplePose.add(new Pose(a)))
                        .collect(Collectors.toList());
                allPoses.clear();
                allPoses.addAll(yellowPoses);
                allPoses.addAll(colorPoses);
            }, t -> !VisionValueStorage.takeFrame || (t - sampleTime > maxDelay));
    }

}