package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.ValueStorage.Side;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;
public class Vision {
    public static final long exposure = 20;
    public static final SimpleMatrix camMat1 = new SimpleMatrix(new double[][] {{0, 0, 0}, {0, 0, 0}, {0, 0, 1}});
    public static final double[] camPos1 = {0, 0, 0};
    private VisionPortal portal;
    private SampleProcessor proc;
    public Vision(Robot robot, CommandOpMode opMode, Side side) {
        proc = new SampleProcessor(side);
        portal = new VisionPortal.Builder()
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "camera"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(proc)
                .build();
        while (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {}
    }
    public void start() {
        portal.getCameraControl(ExposureControl.class).setMode(ExposureControl.Mode.Manual);
        portal.getCameraControl(ExposureControl.class).setExposure(exposure, TimeUnit.MILLISECONDS);
        portal.setProcessorEnabled(proc, false);
        portal.stopLiveView();
    }
    public void close() {
        portal.close();
    }
}