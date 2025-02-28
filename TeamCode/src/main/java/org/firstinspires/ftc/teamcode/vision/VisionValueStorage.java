package org.firstinspires.ftc.teamcode.vision;
import static java.lang.Math.*;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import java.util.ArrayList;
import java.util.List;
public class VisionValueStorage {
    public static volatile boolean takeFrame = true;
    public static volatile List<Point3> yellowPoses;
    public static volatile List<Point3> colorPoses;
    public static final Point center = new Point(0.25, 0.5);
    public static final Point[][] pts = new Point[][]{{new Point(7, 5.1) , new Point(14, 5.8) , new Point(24, 7.4)},
                                                      {new Point(7, 0)   , new Point(14, 0)   , new Point(24, 0)},
                                                      {new Point(7, -5.1), new Point(14, -5.8), new Point(24, -7.4)}};
    public static Point3 camToWorld(Point3 cam) {
        Point p1 = bilinear(new Point(cam.x + 0.01 * cos(cam.z), cam.y + 0.01 * sin(cam.z)), pts);
        Point p2 = bilinear(new Point(cam.x - 0.01 * cos(cam.z), cam.y - 0.01 * sin(cam.z)), pts);
        return new Point3((p1.x + p2.x) / 2, (p1.y + p2.y) / 2, atan2(p1.y - p2.y, p1.x - p2.x));
    }
    private static Point bilinear(Point p, Point[][] pts) {
        double w = pts[0].length - 1;
        double h = pts.length - 1;
        int i = (int)floor(p.x * w);
        int j = (int)floor(p.y * h);
        double fx = p.x * w - i;
        double fy = p.y * h - j;
        double f1 = (1 - fx) * (1 - fy);
        double f2 = fx * (1 - fy);
        double f3 = (1 - fx) * fy;
        double f4 = fx * fy;
        return new Point(f1 * pts[j][i].x + f2 * pts[j][i + 1].x + f3 * pts[j + 1][i].x + f4 * pts[j + 1][i + 1].x,
                         f1 * pts[j][i].y + f2 * pts[j][i + 1].y + f3 * pts[j + 1][i].y + f4 * pts[j + 1][i + 1].y);
    }
}