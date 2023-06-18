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

import java.util.List;

/**
 * Class meant to handle commands from the Ground Data System and execute them
 * in Astrobee
 */

public class YourService extends KiboRpcService {
    private static final int LOOP_LIMIT = 5;
    private final String TAG = this.getClass().getSimpleName();

    // relative to robot body
    private static final float[] NAV_CAM_POSITION = new float[] { 0.1177f, -0.0422f, -0.0826f };
    private static final float[] LASER_POSITION = new float[] { 0.1302f, 0.0572f, -0.1111f };
    private int[] count = { 0, 0, 0, 0, 0, 0, 0 };

    private float markerLength = 0.05f;
    private MatOfPoint3 objPoints = new MatOfPoint3(
            new Point3(-markerLength / 2.0f, markerLength / 2.0f, 0),
            new Point3(markerLength / 2.0f, markerLength / 2.0f, 0),
            new Point3(markerLength / 2.0f, -markerLength / 2.0f, 0),
            new Point3(-markerLength / 2.0f, -markerLength / 2.0f, 0));
    private Mat navCamMatrix = new Mat(3, 3, CvType.CV_32FC1);
    private MatOfDouble distCoeffs;
    private Dictionary arucoDict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

    private void initCameraMatrix() {
        double[][] simData = { { 523.105750, 0.000000, 635.434258 },
                { 0.000000, 534.765913, 500.335102 },
                { 0.000000, 0.000000, 1.000000 } };
        navCamMatrix.put(0, 0, simData[0]);
        navCamMatrix.put(1, 0, simData[1]);
        navCamMatrix.put(2, 0, simData[2]);
    }

    private void initDistCoeffs() {
        double[] simData = { -0.164787, 0.020375, -0.001572, -0.000369, 0.000000 };
        distCoeffs.put(0, 0, simData);
    }

    private void init() {
        initCameraMatrix();
        initDistCoeffs();
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

    private void turnToTarget() {
        Point point = new Point(11.2746d, -9.92284d, 5.2988d);
        Quaternion quaternion = new Quaternion(0.707f, 0.0f, 0.0f, 0.707f);
        api.moveTo(point, quaternion, true);
    }

    private void start() {
        Log.i(TAG, "start mission");
        // the mission starts
        api.startMission();
    }

    @Override
    protected void runPlan1() {
        init();

        start();

        goToPoint1();

        handleTarget(1);

        api.reportMissionCompletion("Mission Complete!");
    }

    @Override
    protected void runPlan2() {
        // write your plan 2 here
    }

    @Override
    protected void runPlan3() {
        // write your plan 3 here
    }

    private void calibrateLocation(int targetNumber) {
        Mat image = api.getMatNavCam();
        List<Mat> corners = null;
        Mat ids = new Mat();
        Aruco.detectMarkers(image, arucoDict, corners, ids);
        Aruco.drawDetectedMarkers(image, corners, ids);

        if (ids.size().empty()) {
            return;
        }

        Mat rvecs = new Mat();
        Mat tvecs = new Mat();
        Aruco.estimatePoseSingleMarkers(corners, markerLength, navCamMatrix, distCoeffs, rvecs, tvecs);
        Aruco.drawAxis(image, navCamMatrix, distCoeffs, rvecs, tvecs, 0.1f);

        api.saveMatImage(image, "target_" + targetNumber + "_" + count[targetNumber] + ".png");
        count[targetNumber]++;

        MatOfPoint2f imagePoints = new MatOfPoint2f();
        MatOfPoint3f objPoints = new MatOfPoint3f();
        objPoints.fromList(this.objPoints.toList());
        Calib3d.projectPoints(objPoints, rvecs, tvecs, navCamMatrix, distCoeffs, imagePoints);
        for (org.opencv.core.Point point: imagePoints.toList()) {
            Imgproc.drawMarker(image, point, new Scalar(0, 255, 0), Imgproc.MARKER_CROSS);
        }

        Point3 targetPoint = new Point3();
        targetPoint.x = (float) tvecs.get(0, 0)[0];
        targetPoint.y = (float) tvecs.get(0, 0)[1];
        targetPoint.z = (float) tvecs.get(0, 0)[2];
        Point3 targetPointInWorld = new Point3();
        targetPointInWorld.x = targetPoint.x + NAV_CAM_POSITION[0];
        targetPointInWorld.y = targetPoint.y + NAV_CAM_POSITION[1];
        targetPointInWorld.z = targetPoint.z + NAV_CAM_POSITION[2];
        Log.i(TAG, "target " + targetNumber + " location: " + targetPointInWorld.x + ", " + targetPointInWorld.y + ", "
                + targetPointInWorld.z);
    }

    private void handleTarget(int targetNumber) {
        calibrateLocation(targetNumber);
        api.laserControl(true);
        api.takeTargetSnapshot(targetNumber);
    }
}
