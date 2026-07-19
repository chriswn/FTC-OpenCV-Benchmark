package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
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

    // Set true only if you want to re-test the "high contrast" synthetic tag instead of a blank frame
    private static final boolean USE_SYNTHETIC_TAG_PATTERN = true;

    @Override
    public void runOpMode() {
        // Declare objects at top scope so they can always be cleaned up in finally
        Mat testFrame = null;
        Mat ids = null;
        ArucoDetector detector = null;
        Dictionary dictionary = null;
        DetectorParameters detectorParams = null;
        List<Mat> corners = new ArrayList<>();

        try {
            telemetry.addData("Status", "OpMode started"); // proves telemetry pipe is alive at all
            telemetry.update();

            if (!OpenCVLoader.initDebug()) {
                telemetry.addLine("Error: OpenCV 5 native library not loaded.");
                telemetry.update();
                RobotLog.ee("OpenCVBenchmark", "OpenCVLoader.initDebug() returned false");
                return; // no point continuing without the native lib
            } else {
                // Force TBB into single-threaded mode to bypass native scheduler deadlocks
                Core.setNumThreads(1);
                telemetry.addLine("OpenCV 5 Core threading restricted to 1.");
            }

            telemetry.addData("Status", "Ready. Press Play to start OpenCV 5 AprilTag benchmark.");
            telemetry.update();

            waitForStart();

            if (!opModeIsActive()) return;

            telemetry.addData("Status", "Initializing OpenCV 5 AprilTag Detector...");
            telemetry.update();

            // 1. Create a mock 640x480 frame in memory
            testFrame = new Mat(480, 640, CvType.CV_8UC1, new Scalar(127));

            if (USE_SYNTHETIC_TAG_PATTERN) {
                // Fake black-and-white square (simulating a basic AprilTag grid)
                Imgproc.rectangle(testFrame, new Point(220, 140), new Point(420, 340), new Scalar(0), -1);
                Imgproc.rectangle(testFrame, new Point(260, 180), new Point(380, 300), new Scalar(255), -1);
                Imgproc.rectangle(testFrame, new Point(300, 220), new Point(340, 260), new Scalar(0), -1);
            }

            // 2. Initialize the AprilTag Detector using OpenCV 5's native ArucoDetector
            dictionary = Objdetect.getPredefinedDictionary(Objdetect.DICT_APRILTAG_36h11);
            detectorParams = new DetectorParameters();
            detector = new ArucoDetector(dictionary, detectorParams);

            ids = new Mat();

            // 3. Warm-up loop
            telemetry.addData("Status", "Warm-up (20 decodes)...");
            telemetry.update();

            for (int i = 0; i < 20; i++) {
                if (!opModeIsActive()) break;
                detector.detectMarkers(testFrame, corners, ids);
                releaseAndClear(corners); // release native Mats before dropping references
            }

            // 4. Timed testing loop (200 frame decodes)
            int iterations = 200;
            telemetry.addData("Status", "Running 200 native AprilTag decodes...");
            telemetry.update();

            long startTime = System.nanoTime();
            int actualIterations = 0;

            for (int i = 0; i < iterations; i++) {
                if (!opModeIsActive()) break;

                detector.detectMarkers(testFrame, corners, ids);
                releaseAndClear(corners);
                actualIterations++;

                // Heartbeat so a stall shows exactly where it happened, instead of nothing at all
                if (i % 20 == 0) {
                    telemetry.addData("Progress", "%d / %d", i, iterations);
                    telemetry.update();
                }
            }

            long endTime = System.nanoTime();

            telemetry.addData("Status", "Loop finished. Iterations completed: " + actualIterations);
            telemetry.update();

            if (actualIterations > 0) {
                double totalMs = (endTime - startTime) / 1_000_000.0;
                double averageMs = totalMs / actualIterations;

                while (opModeIsActive()) {
                    telemetry.addData("Status", "OpenCV 5 AprilTag Benchmark Complete!");
                    telemetry.addData("Method", "Native ArucoDetector (DICT_APRILTAG_36h11)");
                    telemetry.addData("Total Time", "%.2f ms for %d decodes", totalMs, actualIterations);
                    telemetry.addData("Average Decode Latency", "%.4f ms", averageMs);
                    telemetry.addData("Est. AprilTag FPS Limit", "%.1f FPS", 1000.0 / averageMs);
                    telemetry.update();
                    idle();
                }
            } else {
                telemetry.addData("Status", "No iterations completed (stopped before loop ran).");
                telemetry.update();
            }

        } catch (Exception e) {
            // Won't catch a native-level crash/deadlock, but catches any Java-side failure
            telemetry.addData("Java Exception Caught", e.getMessage());
            telemetry.update();
            RobotLog.ee("OpenCVBenchmark", e, "Exception during benchmark");

        } catch (Throwable t) {
            // Catches Errors (e.g. OutOfMemoryError) that a plain Exception catch would miss
            telemetry.addData("Fatal Error Caught", String.valueOf(t.getMessage()));
            telemetry.update();
            RobotLog.ee("OpenCVBenchmark", t, "Fatal error during benchmark");

        } finally {
            releaseAndClear(corners);
            if (ids != null) ids.release();
            if (testFrame != null) testFrame.release();

            detector = null;
            dictionary = null;
            detectorParams = null;

            telemetry.addData("Status", "Native Memory Cleaned Up Safely.");
            telemetry.update();
        }
    }

    /**
     * Releases the native memory backing every Mat in the list, then clears the list.
     * corners.clear() alone only drops Java references -- it never frees the native
     * buffers detectMarkers() allocated, which leaks native memory every call.
     */
    private void releaseAndClear(List<Mat> mats) {
        for (Mat m : mats) {
            if (m != null) {
                m.release();
            }
        }
        mats.clear();
    }
}