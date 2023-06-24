package jp.jaxa.iss.kibo.rpc.defaultapk.Target;

import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point3;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.defaultapk.Target.TargetConfig;

public class TargetManager {
    private static final String TAG = "TargetManager";

    private static void inputPoints(Mat targetPoint, Point3 pos, Mat lineDirection, Mat linePoint) {
        targetPoint.put(0, 0, pos.z + TargetConfig.NAV_CAM_POSITION[0]);
        targetPoint.put(1, 0, pos.x + TargetConfig.NAV_CAM_POSITION[1]);
        targetPoint.put(2, 0, pos.y + TargetConfig.NAV_CAM_POSITION[2]);

        Log.i(TAG, "Relative to center of kibo in its cords: " + targetPoint.dump());
        // (1, 0, 0) represents the direction of the laser which is shoot forward
        lineDirection.put(0, 0, 1);
        lineDirection.put(1, 0, 0);
        lineDirection.put(2, 0, 0);

        for (int i = 0; i < 3; ++i) {
            linePoint.put(i, 0, TargetConfig.LASER_POSITION[i]);
        }
    }

    private static double[] calculateNewOrientation(double[] rotaion, Quaternion originalOrientation) {
        double[] conjugate = { rotaion[0], -rotaion[1], -rotaion[2], -rotaion[3] };
        Log.i(TAG, "quaternion: " + rotaion[0] + ", " + rotaion[1] + ", " + rotaion[2] + ", " + rotaion[3]);

        Log.i(TAG, "original orientation: " + originalOrientation.toString());
        double[] originalOrientationInDouble = { originalOrientation.getW(), originalOrientation.getX(),
                originalOrientation.getY(), originalOrientation.getZ() };
        double[] orientation = multiplyQuaternions(rotaion, multiplyQuaternions(originalOrientationInDouble, conjugate));
        double norm = Math.sqrt(orientation[0] * orientation[0] + orientation[1] * orientation[1]
                + orientation[2] * orientation[2] + orientation[3] * orientation[3]);
        for (int i = 0; i < 4; ++i) {
            orientation[i] /= norm;
        }

        return orientation;
    }

    public static Quaternion calibrateLocation(Mat image, Quaternion originalOrientation) {
        Log.i(TAG, "In calibrateLocation");
        if (image.empty()) {
            Log.i(TAG, "image is empty");
            return new Quaternion(0, 0, 0, 1);
        }

        // this method will check the image and return the ids and corners of the
        // markers
        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();
        Aruco.detectMarkers(image, TargetConfig.arucoDict, corners, ids);

        if (ids.size().empty()) {
            Log.i(TAG, "No markers detected");
            return new Quaternion(0, 0, 0, 1);
        }
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

        Mat targetPoint = new Mat(3, 1, CvType.CV_64FC1);
        Mat lineDirection = new Mat(3, 1, CvType.CV_64FC1);
        Mat linePoint = new Mat(3, 1, CvType.CV_64FC1);
        inputPoints(targetPoint, pos, lineDirection, linePoint);

        // LineRotatoin class will calculate the angle that kibo should turn
        LineRotation lineRotation = new LineRotation(linePoint, lineDirection, targetPoint);
        double[] orientation = calculateNewOrientation(lineRotation.getQuaternion(), originalOrientation);
        Quaternion orientationQuaternion = new Quaternion((float) orientation[1], (float) orientation[2],
                (float) orientation[3], (float) orientation[0]);
        Log.i(TAG, "orientation: " + orientationQuaternion.toString());

        return orientationQuaternion;
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
    private static double[] multiplyQuaternions(double[] q1, double[] q2) {
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
