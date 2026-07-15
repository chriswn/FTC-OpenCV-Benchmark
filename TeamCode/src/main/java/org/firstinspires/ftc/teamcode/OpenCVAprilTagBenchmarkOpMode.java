package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetectorJNI; // standard EasyOpenCV AprilTag wrapper

@Autonomous(name = "OpenCV AprilTag Benchmark", group = "Benchmark")
public class OpenCVAprilTagBenchmarkOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Ready. Press Play to start AprilTag benchmark.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Status", "Initializing AprilTag Solver...");
            telemetry.update();

            // 1. Create a mock 640x480 frame in memory
            Mat testFrame = new Mat(480, 640, CvType.CV_8UC1, new Scalar(127));

            // Draw a fake black-and-white square (simulating a basic AprilTag grid)
            Imgproc.rectangle(testFrame, new Point(220, 140), new Point(420, 340), new Scalar(0), -1);
            Imgproc.rectangle(testFrame, new Point(260, 180), new Point(380, 300), new Scalar(255), -1);
            Imgproc.rectangle(testFrame, new Point(300, 220), new Point(340, 260), new Scalar(0), -1);

            // 2. Initialize the AprilTag Detector (standard tag36h11 used in FTC)
            long nativeDetectorPtr = AprilTagDetectorJNI.createApriltagDetector(
                    AprilTagDetectorJNI.TagFamily.TAG_36h11.string,
                    3.0f, // Decimation
                    4     // Threads
            );

            // 3. Warm up loop
            for (int i = 0; i < 20; i++) {
                AprilTagDetectorJNI.runApriltagDetector(nativeDetectorPtr, testFrame.getNativeObjAddr(), 640, 480);
            }

            // 4. Timed testing loop (Running 200 frame decodes - AprilTag is slow, so 200 is plenty!)
            int iterations = 200;
            telemetry.addData("Status", "Running 200 AprilTag decodes...");
            telemetry.update();

            long startTime = System.nanoTime();
            for (int i = 0; i < iterations; i++) {
                AprilTagDetectorJNI.runApriltagDetector(nativeDetectorPtr, testFrame.getNativeObjAddr(), 640, 480);
            }
            long endTime = System.nanoTime();

            // Calculate benchmark stats
            double totalMs = (endTime - startTime) / 1000000.0;
            double averageMs = totalMs / iterations;

            // Free native pointer resource memory
            AprilTagDetectorJNI.releaseApriltagDetector(nativeDetectorPtr);
            testFrame.release();

            // 5. Output results
            while (opModeIsActive()) {
                telemetry.addData("Status", "AprilTag Benchmark Complete!");
                telemetry.addData("Total Time (200 tags)", "%.2f ms", totalMs);
                telemetry.addData("Average Decode Latency", "%.4f ms", averageMs);
                telemetry.addData("Est. AprilTag FPS Limit", "%.1f FPS", 1000.0 / averageMs);
                telemetry.update();
                idle();
            }
        }
    }
}