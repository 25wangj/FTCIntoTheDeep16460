package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.hardware.ValueStorage.Side;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
public class SampleProcessor implements VisionProcessor {
    public static final Scalar blueLower = new Scalar(0, 0, 150);
    public static final Scalar blueUpper = new Scalar(255, 255, 255);
    public static final Scalar redLower = new Scalar(0, 150, 0);
    public static final Scalar redUpper = new Scalar(255, 255, 255);
    public static final Scalar yellowLower = new Scalar(0, 0, 0);
    public static final Scalar yellowUpper = new Scalar(255, 255, 255);
    private Scalar colorLower;
    private Scalar colorUpper;
    private Mat yellowMask = new Mat();
    private Mat colorMask = new Mat();
    public SampleProcessor(Side side) {
        if (side == Side.BLUE) {
            colorLower = blueLower;
            colorUpper = blueUpper;
        } else {
            colorLower = redLower;
            colorUpper = redUpper;
        }
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {}
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
}
