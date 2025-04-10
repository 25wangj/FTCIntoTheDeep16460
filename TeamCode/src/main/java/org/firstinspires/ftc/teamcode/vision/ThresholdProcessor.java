package org.firstinspires.ftc.teamcode.vision;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2Luv;
import static org.opencv.imgproc.Imgproc.MORPH_CLOSE;
import static org.opencv.imgproc.Imgproc.MORPH_OPEN;
import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.List;

public class ThresholdProcessor implements VisionProcessor {
    public static Scalar lower = new Scalar(0, 0, 0);
    public static Scalar upper = new Scalar(255, 255, 255);
    private int n = 6;
    private Mat mask = new Mat();
    private int w;
    private int h;
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        w = width;
        h = height;
    }
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, mask, COLOR_RGB2Luv);
        Core.inRange(mask, lower, upper, mask);
        Imgproc.morphologyEx(mask, mask, MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(mask, mask, MORPH_CLOSE, new Mat());
        Core.bitwise_not(mask, mask);
        frame.setTo(new Scalar(0, 0, 0), mask);
        for (double i = 1; i < n; i++) {
            Imgproc.line(frame, new Point(0, h*i/n), new Point(w, h*i/n), new Scalar(255, 255, 255), 1);
            Imgproc.line(frame, new Point(w*i/n, 0), new Point(w*i/n, h), new Scalar(255, 255, 255), 1);
        }
        return null;
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
}
