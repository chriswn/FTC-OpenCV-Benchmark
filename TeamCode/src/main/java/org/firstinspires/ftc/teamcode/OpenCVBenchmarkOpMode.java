package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.geometry.Geometry;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "OpenCV Performance Benchmark", group = "Benchmark")
public class OpenCVBenchmarkOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Ready. Press Play to start benchmark.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Status", "Running benchmark...");
            telemetry.update();

            // 1. Set up a mock webcam frame (640x480) in memory
            Mat testFrame = new Mat(480, 640, CvType.CV_8UC3);
            Mat hsvFrame = new Mat();
            Mat thresholded = new Mat();
            List<MatOfPoint> contours = new ArrayList<>();

            // --- SETUP WARP PERSPECTIVE ---
            MatOfPoint2f srcPoints = new MatOfPoint2f(
                    new Point(100, 50),
                    new Point(540, 50),
                    new Point(50, 430),
                    new Point(590, 430)
            );
            MatOfPoint2f dstPoints = new MatOfPoint2f(
                    new Point(0, 0),
                    new Point(640, 0),
                    new Point(0, 480),
                    new Point(640, 480)
            );

            // Use the points directly; the compiler will resolve the correct overload
            // Note: In OpenCV 5.0, getPerspectiveTransform moved to the Geometry class
            Mat warpMatrix = Geometry.getPerspectiveTransform(srcPoints, dstPoints);
            Mat warpedFrame = new Mat();

            // Native memory allocation optimized: define size once!
            Size targetSize = new Size(640, 480);
            Mat contourHierarchy = new Mat();

            // 2. Warm up the processor (JVM optimization)
            for (int i = 0; i < 100; i++) {
                Imgproc.cvtColor(testFrame, hsvFrame, Imgproc.COLOR_BGR2HSV);
                Core.inRange(hsvFrame, new org.opencv.core.Scalar(0, 100, 100), new org.opencv.core.Scalar(10, 255, 255), thresholded);

                // Warps the perspective of the frame
                Imgproc.warpPerspective(testFrame, warpedFrame, warpMatrix, targetSize);

                Imgproc.findContours(thresholded, contours, contourHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
                contours.clear();
            }

            // 3. Timed test loop (1000 frames)
            int iterations = 1000;
            long startTime = System.nanoTime();

            for (int i = 0; i < iterations; i++) {
                // Color conversion
                Imgproc.cvtColor(testFrame, hsvFrame, Imgproc.COLOR_BGR2HSV);

                // Color thresholding
                Core.inRange(hsvFrame, new org.opencv.core.Scalar(0, 100, 100), new org.opencv.core.Scalar(10, 255, 255), thresholded);

                // Warps the perspective of the frame
                Imgproc.warpPerspective(testFrame, warpedFrame, warpMatrix, targetSize);

                // Contour tracking
                Imgproc.findContours(thresholded, contours, contourHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                contours.clear();
            }

            long endTime = System.nanoTime();
            double totalMs = (endTime - startTime) / 1000000.0;
            double averageMs = totalMs / iterations;

            // Free native memory buffers explicitly when done
            testFrame.release();
            hsvFrame.release();
            thresholded.release();
            warpedFrame.release();
            warpMatrix.release();
            contourHierarchy.release();
            srcPoints.release();
            dstPoints.release();

            // 4. Output results to driver station
            while (opModeIsActive()) {
                telemetry.addData("Status", "Benchmark Complete!");
                telemetry.addData("Total Time (1000 frames)", "%.2f ms", totalMs);
                telemetry.addData("Average Per Frame", "%.4f ms", averageMs);
                telemetry.addData("Est. Frame Rate Limit", "%.1f FPS", 1000.0 / averageMs);
                telemetry.update();
                idle();
            }
        }
    }
}