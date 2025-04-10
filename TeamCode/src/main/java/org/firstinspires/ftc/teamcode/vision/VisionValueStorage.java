package org.firstinspires.ftc.teamcode.vision;
import static java.lang.Math.*;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;

import java.util.List;
public class VisionValueStorage {
    public static class VisionValues {
        public Point[][] pts;
        public double minArea;
        public Point p1;
        public Point p2;
        public VisionValues(Point[][] pts, double minArea, Point p1, Point p2) {
            this.pts = pts;
            this.minArea = minArea;
            this.p1 = p1;
            this.p2 = p2;
        }
    }
    public static final VisionValues bucketVals = new VisionValues(
            new Point[][]{{new Point(11.7, 4.9) , new Point(18.5, 6.5) , new Point(27.3, 6.5)},
                          {new Point(11.7, 0)   , new Point(18.5, 0)   , new Point(27.3, 0)},
                          {new Point(11.7, -4.9), new Point(18.5, -6.5), new Point(27.3, -6.5)}},
            0.0125, new Point(0, 0), new Point(1, 1));
    public static final VisionValues chamberVals = new VisionValues(
            new Point[][]{{new Point(14, 7.4) , new Point(20.8, 8.8) , new Point(30.4, 11),  new Point(46.8, 14.5)},
                          {new Point(14, 0)   , new Point(20.8, 0)   , new Point(30.4, 0),   new Point(46.8, 0)},
                          {new Point(14, -7.4), new Point(20.8, -8.8), new Point(30.4, -11), new Point(46.8, -14.5)}},
            0.007, new Point(0.15, 0), new Point(1, 1));
    public static volatile boolean takeFrame = true;
    public static volatile List<Point3> yellowPoints;
    public static volatile List<Point3> colorPoints;
    public static volatile VisionValues vals = bucketVals;
    public static Point3 camToWorld(Point3 cam, Point[][] pts) {
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