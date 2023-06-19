package jp.jaxa.iss.kibo.rpc.defaultapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.core.Core.gemm;

/**
 * Class meant to handle commands from the Ground Data System and execute them
 * in Astrobee
 */

public class YourService extends KiboRpcService {
    private static final int LOOP_LIMIT = 5;
    private final String TAG = this.getClass().getSimpleName();

    // relative to robot body
    private static final float[] NAV_CAM_POSITION = new float[] { 0.1177f,
            -0.0422f, -0.0826f };
    private static final float[] LASER_POSITION = new float[] { 0.1302f, 0.0572f,
            -0.1111f };
    private int[] count = new int[] { 0, 0, 0, 0, 0, 0, 0 };
    private static final float MARKER_LENGTH = 0.05f;

    @Override
    protected void runPlan1() {
        Log.i(TAG, "Running plan 1");

        MatOfPoint3 objPoints = new MatOfPoint3(
                new Point3(-MARKER_LENGTH / 2.0f, MARKER_LENGTH / 2.0f, 0f),
                new Point3(MARKER_LENGTH / 2.0f, MARKER_LENGTH / 2.0f, 0f),
                new Point3(MARKER_LENGTH / 2.0f, -MARKER_LENGTH / 2.0f, 0f),
                new Point3(-MARKER_LENGTH / 2.0f, -MARKER_LENGTH / 2.0f, 0f));
        Mat navCamMatrix = new Mat(3, 3, CvType.CV_32FC1);
        MatOfDouble distCoeffs = new MatOfDouble();
        Dictionary arucoDict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

        init(navCamMatrix, distCoeffs);

        Log.i(TAG, "finish init");

        start();

        goToPoint1();

        handleTarget(1, navCamMatrix, distCoeffs, arucoDict, objPoints);

        api.reportMissionCompletion("Mission Complete!");
    }

    @Override
    protected void runPlan2() {
        Log.i(TAG, "Running plan 2");
        runPlan1();
    }

    @Override
    protected void runPlan3() {
        Log.i(TAG, "Running plan 3");
        runPlan1();
    }

    private void init(Mat navCamMatrix, MatOfDouble distCoeffs) {
        navCamMatrix.put(0, 0, api.getNavCamIntrinsics()[0]);
        distCoeffs.put(0, 0, api.getNavCamIntrinsics()[1]);
    }

    private void goToPoint1() {
        // avoid KOZ
        Point point = new Point(10.3f, -10.2f, 4.32f);
        Quaternion quaternion = new Quaternion(0f, 0f, 0f, 1f);
        api.moveTo(point, quaternion, true);
        // point 1
        point = new Point(11.2746d, -9.92284d, 5.2988d);
        quaternion = new Quaternion(0.0f, 0.0f, -0.707f, 0.707f);
        api.moveTo(point, quaternion, true);
    }

    private void start() {
        Log.i(TAG, "start mission");
        // the mission starts
        api.startMission();
    }

    private void calibrateLocation(int targetNumber, Mat navCamMatrix,
            MatOfDouble distCoeffs, Dictionary arucoDict, MatOfPoint3 objPoints) {
        Log.i(TAG, "In calibrateLocation");
        Mat image = api.getMatNavCam();
        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();
        Aruco.detectMarkers(image, arucoDict, corners, ids);
        Aruco.drawDetectedMarkers(image, corners, ids);

        if (ids.size().empty()) {
            return;
        }

        Mat rvecs = new Mat();
        Mat tvecs = new Mat();
        Log.i(TAG, "Estimating markers pose");
        Aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH, navCamMatrix,
                distCoeffs, rvecs, tvecs);

        // Iterate over each marker and calculate its center point
        for (int i = 0; i < ids.size().height; ++i) {
            Mat rvec = rvecs.row(i);
            Mat tvec = tvecs.row(i);

            // Draw marker axes on the image
            Calib3d.drawFrameAxes(image, navCamMatrix, distCoeffs, rvec, tvec,
                    MARKER_LENGTH / 2.0f);

            // Convert rotation vector to rotation matrix
            Mat rotMat = new Mat();
            Calib3d.Rodrigues(rvec, rotMat);

            // Calculate marker center point
            Mat markerPoint = new Mat();
            gemm(rotMat.inv(), tvec.t(), 1, new Mat(), 0, markerPoint);
            /*
             * error message:
             * 06-19 12:33:25.353 I/KiboRpcApi( 1700): [Start] getMatNavCam
             * 06-19 12:33:25.354 I/KiboRpcApi( 1700): [Finish] getMatNavCam
             * 06-19 12:33:25.384 I/YourService( 1700): Estimating markers pose
             * 06-19 12:33:25.385 E/cv::error()( 1700): OpenCV(3.4.4) Error: Assertion
             * failed (type == B.type()) in void cv::gemm(cv::InputArray, cv::InputArray,
             * double, cv::InputArray, double, cv::OutputArray, int), file
             * /build/3_4-contrib_pack-contrib-android/opencv/modules/core/src/matmul.cpp,
             * line 1558
             * 06-19 12:33:25.385 E/org.opencv.core( 1700): core::gemm_11() caught
             * cv::Exception: OpenCV(3.4.4)
             * /build/3_4-contrib_pack-contrib-android/opencv/modules/core/src/matmul.cpp:
             * 1558: error: (-215:Assertion failed) type == B.type() in function 'void
             * cv::gemm(cv::InputArray, cv::InputArray, double, cv::InputArray, double,
             * cv::OutputArray, int)'
             * 06-19 12:33:25.385 E/KiboRpcApi( 1700): Program Down: Exception:
             * 06-19 12:33:25.385 E/KiboRpcApi( 1700): CvException
             * [org.opencv.core.CvException: cv::Exception: OpenCV(3.4.4)
             * /build/3_4-contrib_pack-contrib-android/opencv/modules/core/src/matmul.cpp:
             * 1558: error: (-215:Assertion failed) type == B.type() in function 'void
             * cv::gemm(cv::InputArray, cv::InputArray, double, cv::InputArray, double,
             * cv::OutputArray, int)'
             * 06-19 12:33:25.385 E/KiboRpcApi( 1700): ]
             * 06-19 12:33:25.385 E/KiboRpcApi( 1700): at org.opencv.core.Core.gemm_1(Native
             * Method)
             * 06-19 12:33:25.385 E/KiboRpcApi( 1700): at
             * org.opencv.core.Core.gemm(Core.java:1682)
             */
            markerPoint = markerPoint.t();

            // Log center point of marker
            Log.i(TAG, "Marker " + ids.get(i, 0)[0] + " center point: "
                    + markerPoint.dump());
        }

        api.saveMatImage(image, "target_" + targetNumber + "_" + count[targetNumber]
                + ".png");
        count[targetNumber]++;
    }

    private void handleTarget(int targetNumber, Mat navCamMatrix,
            MatOfDouble distCoeffs, Dictionary arucoDict, MatOfPoint3 objPoints) {
        calibrateLocation(targetNumber, navCamMatrix, distCoeffs, arucoDict, objPoints);
        api.laserControl(true);
        api.takeTargetSnapshot(targetNumber);
        Mat image = api.getMatNavCam();
        api.saveMatImage(image, "target_" + targetNumber + "_laser.png");
    }
}
