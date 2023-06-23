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
import org.opencv.core.Point3;

import Basic.Vector3D;
import PathCaculation.MapConfig;
import PathCaculation.MapManager;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.core.Core.gemm;
import static org.opencv.core.Core.norm;
import static org.opencv.core.Core.subtract;

/**
 * Class meant to handle commands from the Ground Data System and execute them
 * in Astrobee
 */

public class YourService extends KiboRpcService {
    private final String TAG = this.getClass().getSimpleName();
    private Config config;
    private MapManager mapManager;

    /*
     * This class is used to store the configuration of the mission
     */
    private class Config {
        private final float[] NAV_CAM_POSITION = new float[] { 0.1177f,
                -0.0422f, -0.0826f };
        private final float[] LASER_POSITION = new float[] { 0.1302f, 0.0572f,
                -0.1111f };
        private final float MARKER_LENGTH = 0.05f;
        private int[] count = new int[] { 0, 0, 0, 0, 0, 0, 0 };
        private final int LOOP_LIMIT = 5;
        private Mat navCamMatrix = new Mat(3, 3, CvType.CV_32FC1);
        private MatOfDouble distCoeffs = new MatOfDouble();
        private Dictionary arucoDict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

        private Config() {
            navCamMatrix.put(0, 0, api.getNavCamIntrinsics()[0]);
            distCoeffs.put(0, 0, api.getNavCamIntrinsics()[1]);
        }
    }

    private class Estimation {
        private Mat ids;
        private Mat tvecs;
        // rvecs cannot be used directly, so we need to calculate the rotation matrix
        private Mat rotationMatrix = new Mat();
        private double[][][] rotationMatrixData;

        private Estimation(Mat ids, Mat rvecs, Mat tvecs) {
            this.ids = ids;
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

        /*
         * This method will calculate the position of the target by averaging the
         * position each marker delivers
         * The position of target is calculated by using the position of the marker and
         * the rotation matrix
         * 
         * @return the position of the target
         */
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

    private class LineRotation {
        private Mat linePoint;
        private Mat lineDirection;
        private Mat targetPoint;

        /*
         * This is the constructor of LineRotation class. Note that 3 parms are all in
         * Mat form.
         * 
         * @param linePoint the point on the line
         * 
         * @param lineDirection the direction of the line
         * 
         * @param targetPoint the target point
         */
        private LineRotation(Mat linePoint, Mat lineDirection, Mat targetPoint) {
            this.linePoint = linePoint;
            this.lineDirection = lineDirection;
            this.targetPoint = targetPoint;
        }

        private double[] getQuaternion() {
            // Calculate the vector from line point to target point
            double[] lineVector = new double[3];
            lineVector[0] = targetPoint.get(0, 0)[0] - linePoint.get(0, 0)[0];
            lineVector[1] = targetPoint.get(1, 0)[0] - linePoint.get(1, 0)[0];
            lineVector[2] = targetPoint.get(2, 0)[0] - linePoint.get(2, 0)[0];
            Log.i(TAG, "Relative to laser: " + lineVector[0] + ", " + lineVector[1] + ", "
                    + lineVector[2]);

            // Normalize the line vector
            double lineVectorNorm = Math.sqrt(
                    lineVector[0] * lineVector[0] + lineVector[1] * lineVector[1] + lineVector[2] * lineVector[2]);
            double[] normalizedLineVector = { lineVector[0] / lineVectorNorm, lineVector[1] / lineVectorNorm,
                    lineVector[2] / lineVectorNorm };

            // Calculate the rotation matrix using Rodrigues
            Mat rotationMatrix = new Mat();
            Calib3d.Rodrigues(lineDirection, rotationMatrix);

            // Calculate the dot product between lineDirection and normalizedLineVector
            double dotProduct = rotationMatrix.get(0, 0)[0] * normalizedLineVector[0]
                    + rotationMatrix.get(1, 0)[0] * normalizedLineVector[1]
                    + rotationMatrix.get(2, 0)[0] * normalizedLineVector[2];

            // Calculate the rotation axis using cross product. normalizedLineVector cross
            // rotationMatrix
            double[] rotationAxis = new double[3];
            rotationAxis[0] = normalizedLineVector[1] * rotationMatrix.get(2, 0)[0]
                    - normalizedLineVector[2] * rotationMatrix.get(1, 0)[0];
            rotationAxis[1] = normalizedLineVector[2] * rotationMatrix.get(0, 0)[0] - normalizedLineVector[0]
                    * rotationMatrix.get(2, 0)[0];
            rotationAxis[2] = normalizedLineVector[0] * rotationMatrix.get(1, 0)[0]
                    - normalizedLineVector[1] * rotationMatrix.get(0, 0)[0];
            double rotationAxisNorm = Math.sqrt(rotationAxis[0] * rotationAxis[0]
                    + rotationAxis[1] * rotationAxis[1]
                    + rotationAxis[2] * rotationAxis[2]);
            rotationAxis[0] /= rotationAxisNorm;
            rotationAxis[1] /= rotationAxisNorm;
            rotationAxis[2] /= rotationAxisNorm;

            // Calculate the angle between lineDirection and normalizedLineVector
            double angle = Math.acos(dotProduct);

            // Calculate the quaternion parameters using angle and rotation axis
            double halfAngle = angle / 2;
            double sinHalfAngle = Math.sin(halfAngle);

            double[] quaternion = new double[4];
            quaternion[0] = Math.cos(halfAngle);
            quaternion[1] = rotationAxis[0] * sinHalfAngle;
            quaternion[2] = rotationAxis[1] * sinHalfAngle;
            quaternion[3] = rotationAxis[2] * sinHalfAngle;

            return quaternion;
        }

    }

    private Point convertFromVector3D(Vector3D vector) {
        return new Point(vector.getX(), vector.getY(), vector.getZ());
    }

    @Override
    protected void runPlan1() {
        Log.i(TAG, "Running plan 1");

        config = new Config();
        MapConfig mapConfig = new MapConfig();
        mapManager = new MapManager(mapConfig, 0.1);
        Vector3D from = mapConfig.StartPoint;
        Vector3D to = mapConfig.Point1;
        Vector3D[] result = mapManager.getShortestPath(from, to);

        Log.i(TAG, "finish init");

        start();

        Quaternion currentOrientation = api.getRobotKinematics().getOrientation();
        for (Vector3D point: result) {
            Log.i(TAG, "point: " + point.toString());
            Point pointInPoint = convertFromVector3D(point);
            api.moveTo(pointInPoint, currentOrientation, true);
        }

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

    private void start() {
        Log.i(TAG, "start mission");
        // the mission starts
        api.startMission();
    }

    /*
     * This method will multiply two quaternions using Hamilton product rule
     * 
     * @param q1 the first quaternion
     * 
     * @param q2 the second quaternion
     * note: q1 and q2 should be in the form of (w, x, y, z) and the order matters
     * 
     * @return the result of the multiplication
     */
    private double[] multiplyQuaternions(double[] q1, double[] q2) {
        if (q1.length != 4 || q2.length != 4) {
            Log.i(TAG, "Invalid quaternion length");
            return null;
        }

        double w1 = q1[0];
        double x1 = q1[1];
        double y1 = q1[2];
        double z1 = q1[3];

        double w2 = q2[0];
        double x2 = q2[1];
        double y2 = q2[2];
        double z2 = q2[3];

        double[] result = new double[4];
        result[0] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
        result[1] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
        result[2] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
        result[3] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;

        return result;
    }

    private void calibrateLocation(int targetNumber) {
        Log.i(TAG, "In calibrateLocation");
        Mat image = api.getMatNavCam();
        if (image.empty()) {
            Log.i(TAG, "image is empty");
            return;
        }

        // this method will check the image and return the ids and corners of the
        // markers
        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();
        Aruco.detectMarkers(image, config.arucoDict, corners, ids);

        if (ids.size().empty()) {
            Log.i(TAG, "No markers detected");
            return;
        }
        // below are conditions that markers exist

        // calculate rotation and translation vectors
        Mat rvecs = new Mat();
        Mat tvecs = new Mat();
        Aruco.estimatePoseSingleMarkers(corners, config.MARKER_LENGTH, config.navCamMatrix,
                config.distCoeffs, rvecs, tvecs);
        Log.i(TAG, "rvecs: " + rvecs.dump());
        Log.i(TAG, "tvecs: " + tvecs.dump());

        // calculate the position of the target
        Estimation estimation = new Estimation(ids, rvecs, tvecs);
        Point3 pos = estimation.getEstimatedPos();
        Log.i(TAG, "Relative to camera: " + pos.x + ", " + pos.y + ", " + pos.z);

        Mat targetPoint = new Mat(3, 1, CvType.CV_64FC1);
        targetPoint.put(0, 0, pos.z + config.NAV_CAM_POSITION[0]);
        targetPoint.put(1, 0, pos.x + config.NAV_CAM_POSITION[1]);
        targetPoint.put(2, 0, pos.y + config.NAV_CAM_POSITION[2]);
        Log.i(TAG, "Relative to center of kibo in its cords: " + targetPoint.dump());
        // (1, 0, 0) represents the direction of the laser which is shoot forward
        Mat lineDirection = new Mat(3, 1, CvType.CV_64FC1);
        lineDirection.put(0, 0, 1);
        lineDirection.put(1, 0, 0);
        lineDirection.put(2, 0, 0);
        Mat linePoint = new Mat(3, 1, CvType.CV_64FC1);
        for (int i = 0; i < 3; ++i) {
            linePoint.put(i, 0, config.LASER_POSITION[i]);
        }

        // LineRotatoin class will calculate the angle that kibo should turn
        LineRotation lineRotation = new LineRotation(linePoint, lineDirection, targetPoint);
        double[] rotaion = lineRotation.getQuaternion();
        double[] conjugate = { rotaion[0], -rotaion[1], -rotaion[2], -rotaion[3] };
        Log.i(TAG, "quaternion: " + rotaion[0] + ", " + rotaion[1] + ", " + rotaion[2] + ", " + rotaion[3]);

        Quaternion originalOrientation = api.getRobotKinematics().getOrientation();
        Log.i(TAG, "original orientation: " + originalOrientation.toString());
        double[] originalOrientationInDouble = { originalOrientation.getW(), originalOrientation.getX(),
                originalOrientation.getY(), originalOrientation.getZ() };
        double[] orientation = multiplyQuaternions(rotaion, multiplyQuaternions(originalOrientationInDouble, conjugate));
        double norm = Math.sqrt(orientation[0] * orientation[0] + orientation[1] * orientation[1]
                + orientation[2] * orientation[2] + orientation[3] * orientation[3]);
        for (int i = 0; i < 4; ++i) {
            orientation[i] /= norm;
        }
        if (orientation == null) {
            Log.i(TAG, "totalRotation is null");
            return;
        }
        Quaternion orientationQuaternion = new Quaternion((float) orientation[1], (float) orientation[2],
                (float) orientation[3], (float) orientation[0]);
        Log.i(TAG, "orientation: " + orientationQuaternion.toString());

        api.saveMatImage(image, "target_" + targetNumber + "_" + config.count[targetNumber]
                + ".png");
        config.count[targetNumber]++;

        api.relativeMoveTo(new Point(0, 0, 0), orientationQuaternion, true);
    }

    private void handleTarget(int targetNumber) {
        calibrateLocation(targetNumber);
        api.laserControl(true);
        api.takeTargetSnapshot(targetNumber);
    }
}
