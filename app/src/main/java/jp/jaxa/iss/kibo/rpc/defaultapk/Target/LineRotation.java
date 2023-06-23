package jp.jaxa.iss.kibo.rpc.defaultapk.Target;

import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.calib3d.Calib3d;

public class LineRotation {
    private final String TAG = "LineRotation";
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
    public LineRotation(Mat linePoint, Mat lineDirection, Mat targetPoint) {
        this.linePoint = linePoint;
        this.lineDirection = lineDirection;
        this.targetPoint = targetPoint;
    }

    public double[] getQuaternion() {
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
