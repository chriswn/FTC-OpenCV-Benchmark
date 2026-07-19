package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.ArucoDetector;
import org.opencv.objdetect.DetectorParameters; 
import org.opencv.objdetect.Dictionary;
import org.opencv.objdetect.Objdetect;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "OpenCV 5 AprilTag Benchmark", group = "Benchmark")
public class OpenCVAprilTagBenchmarkOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Declare objects at the top scope so they can be cleaned up in the finally block
        Mat testFrame = null;
        Mat ids = null;
        ArucoDetector detector = null;
        Dictionary dictionary = null;
        DetectorParameters detectorParams = null; 

        try {
            if (!OpenCVLoader.initDebug()) {
                telemetry.addLine("Error: OpenCV 5 native library not loaded.");
                telemetry.update();
            }

            telemetry.addData("Status", "Ready. Press Play to start OpenCV 5 AprilTag benchmark.");
            telemetry.update();

            waitForStart();

            if (!opModeIsActive()) return;

            telemetry.addData("Status", "Initializing OpenCV 5 AprilTag Detector...");
            telemetry.update();

            // 1. Create a mock 640x480 frame in memory
            testFrame = new Mat(480, 640, CvType.CV_8UC1, new Scalar(127));

            // Draw a fake black-and-white square (simulating a basic AprilTag grid)
            Imgproc.rectangle(testFrame, new Point(220, 140), new Point(420, 340), new Scalar(0), -1);
            Imgproc.rectangle(testFrame, new Point(260, 180), new Point(380, 300), new Scalar(255), -1);
            Imgproc.rectangle(testFrame, new Point(300, 220), new Point(340, 260), new Scalar(0), -1);

            // 2. Initialize the AprilTag Detector using OpenCV 5's native ArucoDetector
            dictionary = Objdetect.getPredefinedDictionary(Objdetect.DICT_APRILTAG_36h11);
            

            detectorParams = new DetectorParameters(); 
            
            // Explicitly call the JNI setter configurations to initialize thresholds.
            // This prevents uninitialized native parameters from causing infinite/deadlocked threading loops.
            detectorParams.set_adaptiveThreshWinSizeMin(3);
            detectorParams.set_adaptiveThreshWinSizeMax(23);
            detectorParams.set_adaptiveThreshWinSizeStep(10);
            
            detector = new ArucoDetector(dictionary, detectorParams);


            List<Mat> corners = new ArrayList<>();
            ids = new Mat();

            // 3. Warm up loop
            for (int i = 0; i < 20; i++) {
                if (!opModeIsActive()) break; 
                detector.detectMarkers(testFrame, corners, ids);
                corners.clear();
            }

            // 4. Timed testing loop (Running 200 frame decodes)
            int iterations = 200;
            telemetry.addData("Status", "Running 200 native AprilTag decodes...");
            telemetry.update();

            long startTime = System.nanoTime();
            int actualIterations = 0;
            for (int i = 0; i < iterations; i++) {
                if (!opModeIsActive()) break; 
                detector.detectMarkers(testFrame, corners, ids);
                corners.clear();
                actualIterations++;
            }
            long endTime = System.nanoTime();

            if (actualIterations > 0) {
                // Calculate benchmark stats
                double totalMs = (endTime - startTime) / 1000000.0;
                double averageMs = totalMs / actualIterations;

                // 5. Output results
                while (opModeIsActive()) {
                    telemetry.addData("Status", "OpenCV 5 AprilTag Benchmark Complete!");
                    telemetry.addData("Method", "Native ArucoDetector (DICT_APRILTAG_36h11)");
                    telemetry.addData("Total Time", "%.2f ms for %d decodes", totalMs, actualIterations);
                    telemetry.addData("Average Decode Latency", "%.4f ms", averageMs);
                    telemetry.addData("Est. AprilTag FPS Limit", "%.1f FPS", 1000.0 / averageMs);
                    telemetry.update();
                    idle();
                }
            }

        } finally {
            // --- THE CRITICAL NATIVE CLEANUP GUARD ---
            if (ids != null) {
                ids.release();
            }
            if (testFrame != null) {
                testFrame.release();
            }

            detector = null;
            dictionary = null;
            detectorParams = null; 

            telemetry.addData("Status", "Native Memory Cleaned Up Safely.");
            telemetry.update();
        }
    }
}
