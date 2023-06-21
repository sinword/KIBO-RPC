package jp.jaxa.iss.kibo.rpc.defaultapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import android.support.compat.R.id;
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
    private final String TAG = this.getClass().getSimpleName();
    private Config config;

    // relative to robot body

    private class Config {
        private final float[] NAV_CAM_POSITION = new float[] { 0.1177f,
                -0.0422f, -0.0826f };
        private final float[] LASER_POSITION = new float[] { 0.1302f, 0.0572f,
                -0.1111f };
        private final float MARKER_LENGTH = 0.05f;
        private int[] count = new int[] { 0, 0, 0, 0, 0, 0, 0 };
        private final int LOOP_LIMIT = 5;
        private MatOfPoint3 objPoints = new MatOfPoint3(
                new Point3(-MARKER_LENGTH / 2.0f, MARKER_LENGTH / 2.0f, 0f),
                new Point3(MARKER_LENGTH / 2.0f, MARKER_LENGTH / 2.0f, 0f),
                new Point3(MARKER_LENGTH / 2.0f, -MARKER_LENGTH / 2.0f, 0f),
                new Point3(-MARKER_LENGTH / 2.0f, -MARKER_LENGTH / 2.0f, 0f));
        private Mat navCamMatrix = new Mat(3, 3, CvType.CV_32FC1);
        private MatOfDouble distCoeffs = new MatOfDouble();
        private Dictionary arucoDict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

        private void init() {
            navCamMatrix.put(0, 0, api.getNavCamIntrinsics()[0]);
            distCoeffs.put(0, 0, api.getNavCamIntrinsics()[1]);
        }
    }

    private class Estimation {
        private Mat ids;
        private Mat rvecs;
        private Mat tvecs;
        private Mat rotationMatrix = new Mat();
        private double[][][] rotationMatrixData;

        private void inputData(Mat ids, Mat rvecs, Mat tvecs) {
            this.ids = ids;
            this.rvecs = rvecs;
            this.tvecs = tvecs;
            int dataRows = ids.rows();
            rotationMatrixData = new double[dataRows][3][3];
            for (int i = 0; i < dataRows; i++) {
                Mat rotationMatrix = new Mat();
                Calib3d.Rodrigues(rvecs.row(i), rotationMatrix);
                rotationMatrix.get(0, 0, rotationMatrixData[i][0]);
                rotationMatrix.get(1, 0, rotationMatrixData[i][1]);
                rotationMatrix.get(2, 0, rotationMatrixData[i][2]);
            }
        }

        private Point3 getEstimatedPos() {
            Point3 pos = new Point3(0, 0, 0);
            Log.i(TAG, "In getEstimatedPos");
            Log.i(TAG, "ids: " + ids.dump());
            Log.i(TAG, "tvecs: " + tvecs.dump());

            for (int i = 0; i < tvecs.rows(); i++) {
                int id = (int) ids.get(i, 0)[0];
                Log.i(TAG, "id: " + id);
                double[] data = new double[3];
                tvecs.get(i, 0, data);

                Log.i(TAG, "size:" + data.length);
                Log.i(TAG, "data: " + data[0] + ", " + data[1] + ", " + data[2]);
                pos.x += data[0];
                pos.y += data[1];
                pos.z += data[2];
                switch (id % 4) {
                    case 1:
                        // dx = -0.1, dy = 0.0375
                        pos.x += -0.1f * rotationMatrixData[i][0][0] + 0.0375f
                                * rotationMatrixData[i][0][1];
                        pos.y += -0.1f * rotationMatrixData[i][1][0] + 0.0375f
                                * rotationMatrixData[i][1][1];
                        pos.z += -0.1f * rotationMatrixData[i][2][0] + 0.0375f
                                * rotationMatrixData[i][2][1];
                        break;
                    case 2:
                        // dx = 0.1, dy = 0.0375
                        pos.x += 0.1f * rotationMatrixData[i][0][0] + 0.0375f
                                * rotationMatrixData[i][0][1];
                        pos.y += 0.1f * rotationMatrixData[i][1][0] + 0.0375f
                                * rotationMatrixData[i][1][1];
                        pos.z += 0.1f * rotationMatrixData[i][2][0] + 0.0375f
                                * rotationMatrixData[i][2][1];
                        break;
                    case 3:
                        // dx = 0.1, dy = -0.0375
                        pos.x += 0.1f * rotationMatrixData[i][0][0] - 0.0375f
                                * rotationMatrixData[i][0][1];
                        pos.y += 0.1f * rotationMatrixData[i][1][0] - 0.0375f
                                * rotationMatrixData[i][1][1];
                        pos.z += 0.1f * rotationMatrixData[i][2][0] - 0.0375f
                                * rotationMatrixData[i][2][1];
                        break;
                    case 0:
                        // dx = -0.1, dy = -0.0375
                        pos.x += -0.1f * rotationMatrixData[i][0][0] - 0.0375f
                                * rotationMatrixData[i][0][1];
                        pos.y += -0.1f * rotationMatrixData[i][1][0] - 0.0375f
                                * rotationMatrixData[i][1][1];
                        pos.z += -0.1f * rotationMatrixData[i][2][0] - 0.0375f
                                * rotationMatrixData[i][2][1];
                        break;
                }
            }

            pos.x /= tvecs.rows();
            pos.y /= tvecs.rows();
            pos.z /= tvecs.rows();

            return pos;
        }
    }

    @Override
    protected void runPlan1() {
        Log.i(TAG, "Running plan 1");

        config = new Config();
        config.init();

        Log.i(TAG, "finish init");

        start();

        goToPoint1();

        handleTarget(1);

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

    private void calibrateLocation(int targetNumber) {
        Log.i(TAG, "In calibrateLocation");
        Mat image = api.getMatNavCam();
        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();
        Aruco.detectMarkers(image, config.arucoDict, corners, ids);
        Aruco.drawDetectedMarkers(image, corners, ids);

        if (ids.size().empty()) {
            return;
        }

        Mat rvecs = new Mat();
        Mat tvecs = new Mat();
        Log.i(TAG, "Estimating markers pose");
        Aruco.estimatePoseSingleMarkers(corners, config.MARKER_LENGTH, config.navCamMatrix,
                config.distCoeffs, rvecs, tvecs);

        Log.i(TAG, "rvecs: " + rvecs.dump());
        Log.i(TAG, "tvecs: " + tvecs.dump());

        Estimation estimation = new Estimation();
        estimation.inputData(ids, rvecs, tvecs);
        Point3 pos = estimation.getEstimatedPos();

        Log.i(TAG, "Relative position: " + pos.x + ", " + pos.y + ", " + pos.z);

        api.saveMatImage(image, "target_" + targetNumber + "_" + config.count[targetNumber]
                + ".png");
        config.count[targetNumber]++;
    }

    private void handleTarget(int targetNumber) {
        calibrateLocation(targetNumber);
        api.laserControl(true);
        api.takeTargetSnapshot(targetNumber);
    }
}
