package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core; // --- 1. IMPORT THE CORE UTILITY UNIT ---
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.objdetect.ArucoDetector;
import org.opencv.objdetect.DetectorParameters; 
import org.opencv.objdetect.Dictionary;
import org.opencv.objdetect.Objdetect;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "OpenCV 5 AprilTag Isolation Test", group = "Benchmark")
public class OpenCVAprilTagBenchmarkOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        Mat testFrame = null;
        Mat ids = null;
        ArucoDetector detector = null;
        Dictionary dictionary = null;
        DetectorParameters detectorParams = null; 

        try {
            if (!OpenCVLoader.initDebug()) {
                telemetry.addLine("Error: OpenCV 5 native library not loaded.");
                telemetry.update();
            } else {
                // Force single threading to attempt bypass of TBB scheduler deadlock
                Core.setNumThreads(1);
            }

            telemetry.addData("Status", "Ready. Press Play.");
            telemetry.update();

            waitForStart();

            if (!opModeIsActive()) return;

            // --- ISOLATION: A completely blank, single-channel black matrix ---
            // If the detector still freezes on this, TBB's scheduler framework is 100% deadlocked.
            testFrame = new Mat(480, 640, CvType.CV_8UC1, new org.opencv.core.Scalar(0));

            dictionary = Objdetect.getPredefinedDictionary(Objdetect.DICT_APRILTAG_36h11);
            detectorParams = new DetectorParameters(); 
            detector = new ArucoDetector(dictionary, detectorParams);

            List<Mat> corners = new ArrayList<>();
            ids = new Mat();

            telemetry.addData("Status", "Attempting exactly ONE single decode...");
            telemetry.update();

            // Run a single isolated call
            detector.detectMarkers(testFrame, corners, ids);

            telemetry.addData("Status", "SUCCESS! Native call returned without deadlocking.");
            telemetry.update();

            while (opModeIsActive()) {
                idle();
            }

        } catch (Exception e) {
            telemetry.addData("Java Exception Caught", e.getMessage());
            telemetry.update();
        } finally {
            if (ids != null) ids.release();
            if (testFrame != null) testFrame.release();
        }
    }
}
