package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;
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

            // 2. Warm up the processor (JVM optimization)
            for (int i = 0; i < 100; i++) {
                Imgproc.cvtColor(testFrame, hsvFrame, Imgproc.COLOR_BGR2HSV);
                Core.inRange(hsvFrame, new org.opencv.core.Scalar(0, 100, 100), new org.opencv.core.Scalar(10, 255, 255), thresholded);
                Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
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

                // Contour tracking
                Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                contours.clear();
            }

            long endTime = System.nanoTime();
            double totalMs = (endTime - startTime) / 1000000.0;
            double averageMs = totalMs / iterations;

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