package org.firstinspires.ftc.teamcode.vision;
import static org.opencv.imgproc.Imgproc.*;
import static java.lang.Math.*;
import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.hardware.Vision;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
public class SampleProcessor implements VisionProcessor {
    public static final Scalar yellowLower = new Scalar(150, 0, 150);
    public static final Scalar yellowUpper = new Scalar(255, 125, 255);
    public static final Scalar redLower = new Scalar(0, 125, 0);
    public static final Scalar redUpper = new Scalar(255, 255, 255);
    public static final Scalar blueLower = new Scalar(0, 0, 0);
    public static final Scalar blueUpper = new Scalar(255, 255, 110);
    public static final double minArea = 0.0125;
    public static final Size size = new Size(160, 120);
    private Telemetry telemetry = null;
    private Scalar colorLower = blueLower;
    private Scalar colorUpper = blueUpper;
    private Mat luv = new Mat();
    private Mat yellowMask = new Mat();
    private Mat colorMask = new Mat();
    private List<MatOfPoint> yellowContours = new ArrayList<>();
    private List<MatOfPoint> colorContours = new ArrayList<>();
    public SampleProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
        VisionValueStorage.takeFrame = true;
    }
    public SampleProcessor(boolean red) {
        if (red) {
            colorLower = redLower;
            colorUpper = redUpper;
        }
        VisionValueStorage.takeFrame = true;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {}
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (VisionValueStorage.takeFrame) {
            System.out.println(System.nanoTime()*1e-9 - Vision.sampleTime);
            Imgproc.resize(frame, luv, size);
            Imgproc.cvtColor(luv, luv, COLOR_RGB2Luv);
            VisionValueStorage.yellowPoses = blocks(frame, yellowMask, yellowContours, yellowLower, yellowUpper, new Scalar(0, 255, 0));
            VisionValueStorage.colorPoses = blocks(frame, colorMask, colorContours, colorLower, colorUpper, new Scalar(255, 0, 255));
            if (telemetry == null) {
                VisionValueStorage.takeFrame = false;
            } else {
                telemetry.update();
            }
        }
        return null;
    }
    private List<Point3> blocks(Mat frame, Mat mask, List<MatOfPoint> contours, Scalar lower, Scalar upper, Scalar color) {
        Core.inRange(luv, lower, upper, mask);
        Imgproc.morphologyEx(mask, mask, MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(mask, mask, MORPH_CLOSE, new Mat());
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        List<Point3> poses = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            double contourArea = rect.size.width * rect.size.height / (size.width * size.height);
            if (contourArea > minArea) {
                double angle = rect.angle * PI / 180 + (rect.size.width < rect.size.height ? PI / 2 : 0);
                poses.add(new Point3(rect.center.x / size.width, rect.center.y / size.height, angle));
                if (telemetry != null) {
                    Point[] pts = new Point[4];
                    double f = frame.cols() / size.width;
                    rect.points(pts);
                    for (int i = 0; i < 4; i++) {
                        Imgproc.line(frame, scale(pts[i], f), scale(pts[(i + 1) % 4], f), color, 2);
                    }
                    Imgproc.putText(frame, String.format("%.3f %d", contourArea, (int)(angle*180/PI)), scale(rect.center, f), 1, 1, color);
                }
            }
        }
        poses.sort(Comparator.comparingDouble(a -> pow(a.x - VisionValueStorage.center.x, 2) + pow(a.x - VisionValueStorage.center.y, 2)));
        poses.replaceAll(VisionValueStorage::camToWorld);
        if (telemetry != null) {
            for (Point3 p : poses) {
                telemetry.addLine(p.x + " " + p.y + " " + p.z);
            }
        }
        contours.clear();
        return poses;
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
    private Point scale(Point p, double a) {
        return new Point(a * p.x, a * p.y);
    }
}
