package jp.jaxa.iss.kibo.rpc.defaultapk.Target;

import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point3;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.defaultapk.Target.LineRotation;
import jp.jaxa.iss.kibo.rpc.defaultapk.Target.TargetConfig;

import static org.opencv.core.Core.gemm;
import static org.opencv.core.Core.invert;

public class TargetManager {
    private static final String TAG = "TargetManager";

    private static void inputPoints(double[] targetPoint, Point3 pos) {
        targetPoint[0] = pos.z + TargetConfig.NAV_CAM_POSITION[0];
        targetPoint[1] = pos.x + TargetConfig.NAV_CAM_POSITION[1];
        targetPoint[2] = pos.y + TargetConfig.NAV_CAM_POSITION[2];
        Log.i(TAG, "Relative to center of kibo in its cords: " + targetPoint[0] + ", " + targetPoint[1] + ", "
                + targetPoint[2]);
    }

    private static double[] calculateNewOrientation(double[] rotaion, Quaternion originalOrientation) {
        Log.i(TAG, "original orientation: " + originalOrientation.toString());
        double[] originalOrientationInDouble = { originalOrientation.getW(), originalOrientation.getX(),
                originalOrientation.getY(), originalOrientation.getZ() };
        Log.i(TAG, "original rotation: " + rotaion[0] + ", " + rotaion[1] + ", " + rotaion[2] + ", " + rotaion[3]);
        rotaion = LineRotation.convertToRealWorldRotation(rotaion, originalOrientationInDouble);
        Log.i(TAG,
                "quaternion in real world: " + rotaion[0] + ", " + rotaion[1] + ", " + rotaion[2] + ", " + rotaion[3]);

        double[] orientation = multiplyQuaternions(originalOrientationInDouble, rotaion);
        orientation = LineRotation.normalize(orientation);

        return orientation;
    }

    public static double[] calibrateLocation(Mat image, Quaternion originalOrientation) {
        Log.i(TAG, "In calibrateLocation");

        // this method will check the image and return the ids and corners of the
        // markers
        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();
        Aruco.detectMarkers(image, TargetConfig.arucoDict, corners, ids);
        // below are conditions that markers exist

        // calculate rotation and translation vectors
        Mat rvecs = new Mat();
        Mat tvecs = new Mat();
        Aruco.estimatePoseSingleMarkers(corners, TargetConfig.MARKER_LENGTH, TargetConfig.navCamMatrix,
                TargetConfig.distCoeffs, rvecs, tvecs);
        Log.i(TAG, "rvecs: " + rvecs.dump());
        Log.i(TAG, "tvecs: " + tvecs.dump());

        // calculate the position of the target
        Estimation estimation = new Estimation(ids, rvecs, tvecs);
        Point3 pos = estimation.getEstimatedPos();
        Log.i(TAG, "Relative to camera: " + pos.x + ", " + pos.y + ", " + pos.z);

        double[] targetPoint = new double[3];
        inputPoints(targetPoint, pos);
        double[] d = { 0, targetPoint[1] - TargetConfig.LASER_POSITION[1],
                targetPoint[2] - TargetConfig.LASER_POSITION[2] };
        Log.i(TAG, "d: " + d[0] + ", " + d[1] + ", " + d[2]);

        double[] originalOrientationInDouble = { originalOrientation.getW(), originalOrientation.getX(),
                originalOrientation.getY(), originalOrientation.getZ() };
        Mat rotationMatrix = quaternionToRotationMatrix(originalOrientationInDouble);
        Mat vectorInOriginalSystem = transformVector(rotationMatrix, d);
        double[] vectorInOriginalSystemArray = new double[3];
        for (int i = 0; i < 3; i++) {
            vectorInOriginalSystemArray[i] = vectorInOriginalSystem.get(i, 0)[0];
        }
        // // LineRotatoin class will calculate the angle that kibo should turn
        // double[] orientation = calculateNewOrientation(
        // LineRotation.getQuaternion(targetPoint), originalOrientation);F
        // Quaternion orientationQuaternion = new Quaternion((float) orientation[1],
        // (float) orientation[2],
        // (float) orientation[3], (float) orientation[0]);
        // Log.i(TAG, "orientation: " + orientationQuaternion.toString());

        return vectorInOriginalSystemArray;
    }

    public static Mat quaternionToRotationMatrix(double[] orientation) {
        // Create a 3x3 rotation matrix
        Mat rotationMatrix = new Mat(3, 3, CvType.CV_64FC1);

        // Convert quaternion to rotation matrix
        double x = orientation[0];
        double y = orientation[1];
        double z = orientation[2];
        double w = orientation[3];

        rotationMatrix.put(0, 0, 1 - 2 * (y * y + z * z));
        rotationMatrix.put(0, 1, 2 * (x * y - z * w));
        rotationMatrix.put(0, 2, 2 * (x * z + y * w));

        rotationMatrix.put(1, 0, 2 * (x * y + z * w));
        rotationMatrix.put(1, 1, 1 - 2 * (x * x + z * z));
        rotationMatrix.put(1, 2, 2 * (y * z - x * w));

        rotationMatrix.put(2, 0, 2 * (x * z - y * w));
        rotationMatrix.put(2, 1, 2 * (y * z + x * w));
        rotationMatrix.put(2, 2, 1 - 2 * (x * x + y * y));

        return rotationMatrix;
    }

    public static Mat transformVector(Mat rotationMatrix, double[] vector) {
        // Create a 3x1 column vector
        Mat vectorMat = new Mat(3, 1, CvType.CV_64FC1);
        vectorMat.put(0, 0, vector[0]);
        vectorMat.put(1, 0, vector[1]);
        vectorMat.put(2, 0, vector[2]);

        // Transform the vector by multiplying with the rotation matrix
        Mat transformedVector = new Mat();
        gemm(rotationMatrix, vectorMat, 1, new Mat(), 0, transformedVector);

        return transformedVector;
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
    public static double[] multiplyQuaternions(double[] q1, double[] q2) {
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
}
