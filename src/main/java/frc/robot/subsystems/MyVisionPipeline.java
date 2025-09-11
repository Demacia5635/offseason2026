package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;

public class MyVisionPipeline implements VisionPipeline {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("vision");
    NetworkTableEntry targetFound = table.getEntry("target_found");
    NetworkTableEntry targetX = table.getEntry("target_x");
    NetworkTableEntry targetY = table.getEntry("target_y");

    @Override
    public void process(Mat input) {
        Mat output = new Mat();
        Imgproc.cvtColor(input, output, Imgproc.COLOR_BGR2HSV);

        boolean found = false;
        double x = 0;
        double y = 0;
        // calculate target psition

        // set result
        targetFound.setBoolean(found);
        targetX.setDouble(x);
        targetY.setDouble(y);

    }

    public static void detectBall() {
        String fileName = "detectBallExercise.jpg";
        System.load("D:\\Projects\\OpenCV\\opencv\\build\\java\\x64\\opencv_java4100.dll");
        Mat img = Imgcodecs.imread(fileName);
        Mat resized = new Mat();
        Imgproc.resize(img, resized, new Size(320, 240));
        Mat blur = new Mat();
        Imgproc.GaussianBlur(resized, blur, new Size(15, 15), 2);
        HighGui.imshow("blur", blur);
        Mat out = resized.clone();
        HighGui.imshow("original", resized);
        Mat hsv = new Mat();
        Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_BGR2HSV);
        Scalar min = new Scalar(20, 100, 100);
        Scalar max = new Scalar(30, 255, 255);

        Mat mask = new Mat();
        Core.inRange(hsv, min, max, mask);
        HighGui.imshow("mask", mask);
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Mat filterred = new Mat();
        Point anchor = new Point(-1, -1);
        Imgproc.erode(mask, filterred, kernel, anchor, 2);
        Imgproc.dilate(filterred, filterred, kernel, anchor, 2);
        Imgproc.erode(filterred, filterred, kernel, anchor, 4);
        HighGui.imshow("filtter", filterred);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(filterred, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        System.out.println("Number of contours found: " + contours.size());
        MatOfPoint best = null;
        Rect bestRect = null;
        double bestSize = 0;
        double bestRatio = 0;
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint m = contours.get(i);
            double area = Imgproc.contourArea(m);
            if (area > 200) {
                Rect rect = Imgproc.boundingRect(m);
                double r = (double)rect.height / rect.width;
                Imgproc.drawContours(out, contours, i, new Scalar(0, 0, 255), 2);
                Imgproc.rectangle(out, rect, new Scalar(255,0,0),1);
                System.out.println("r = " + r + " area=" + area + " rect=" + rect.height + " / " + rect.width);
                if (best == null || isBetter(r, area, bestRatio, bestSize)) {
                        best = m;
                        bestRect = rect;
                        bestRatio = r;
                        bestSize = area;
                        System.out.println("r = " + r + " area=" + area);
                }
            }
        }
        if (bestRect != null) {
            Imgproc.rectangle(out, bestRect, new Scalar(0, 255, 0), 3);
            Moments m = Imgproc.moments(best);
            int centerX = (int) (m.get_m10() / m.get_m00());
            int centerY = (int) (m.get_m01() / m.get_m00());
            Imgproc.circle(out, new Point(centerX, centerY), 5, new Scalar(0, 255, 0), -1);
        }
        HighGui.imshow("out", out);

        HighGui.waitKey(0);
        HighGui.destroyAllWindows();
        System.exit(1);
    }

    private static boolean isBetter(double r1, double area1, double r2, double area2) {
        if(r1 > 1.0/1.5 && r1 < 1.5 && (r2 < 1.0/1.5 || r2 > 1.5)) {
            return true;
        }
        if(r2 > 1.0/1.5 && r2 < 1.5 && (r1 < 1.0/1.5 || r1 > 1.5)) {
            return false;
        }
        return area1 > area2;
    }


    public static void main(String[] args) {
        detectBall();
        System.exit(1);
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setExposureAuto();
        camera.setFPS(30);
        camera.setResolution(320, 240);
        VisionThread visionThread = new VisionThread(camera, new MyVisionPipeline(), pipeline -> {        });
        visionThread.setDaemon(true);
        visionThread.start();
    }
}
