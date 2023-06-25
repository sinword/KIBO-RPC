package jp.jaxa.iss.kibo.rpc.defaultapk.Target;

import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.CvType;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;

public class TargetConfig {
    public static final float[] NAV_CAM_POSITION = new float[] { 0.1177f,
            -0.0422f, -0.0826f };
    public static final float[] LASER_POSITION = new float[] { 0.1302f, 0.0572f,
            -0.1111f };
    public static final float[] LASER_DIRECTION = new float[] { 1, 0, 0 };
    public static final float MARKER_LENGTH = 0.05f;
    public static int[] count = new int[] { 0, 0, 0, 0, 0, 0, 0 };
    public static Mat navCamMatrix = new Mat(3, 3, CvType.CV_32FC1);
    public static MatOfDouble distCoeffs = new MatOfDouble();
    public static Dictionary arucoDict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

    public TargetConfig(double[][] namCamArray) {
        navCamMatrix.put(0, 0, namCamArray[0]);
        distCoeffs.put(0, 0, namCamArray[1]);
    }
}
