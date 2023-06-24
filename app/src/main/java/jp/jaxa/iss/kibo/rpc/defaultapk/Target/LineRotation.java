package jp.jaxa.iss.kibo.rpc.defaultapk.Target;

import android.support.compat.R.string;
import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.calib3d.Calib3d;

public class LineRotation {
        private static final String TAG = "LineRotation";

        private static double[] normalize(double[] vector) {
                double norm = Math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
                double[] normalizedVector = { vector[0] / norm, vector[1] / norm, vector[2] / norm };
                return normalizedVector;
        }

        private static double[] getRotatedPoint(float[] linePoint, float[] lineDirection, double[] targetPoint) {
                double a = lineDirection[0] * lineDirection[0] + lineDirection[1] * lineDirection[1]
                                + lineDirection[2] * lineDirection[2];
                double b = 2 * (lineDirection[0] * linePoint[0] + lineDirection[1] * linePoint[1]
                                + lineDirection[2] * linePoint[2]);
                double c = linePoint[0] * linePoint[0] + linePoint[1] * linePoint[1] + linePoint[2] * linePoint[2];
                double distance = Math.sqrt(targetPoint[0] * targetPoint[0] + targetPoint[1] * targetPoint[1]
                                + targetPoint[2] * targetPoint[2]);
                c = c - distance * distance;
                double discriminant = b * b - 4 * a * c;
                double t1 = (-b + Math.sqrt(discriminant)) / (2 * a);
                double[] point1 = { linePoint[0] + t1 * lineDirection[0], linePoint[1] + t1 * lineDirection[1],
                                linePoint[2] + t1 * lineDirection[2] };
                return point1;
        }

        public static double[] getQuaternion(double[] targetPoint) {
                double[] rotatedPoint = getRotatedPoint(TargetConfig.LASER_POSITION, TargetConfig.LASER_DIRECTION,
                                targetPoint);
                // Calculate the vector from line point to target point
                double[] lineVector = new double[3];
                lineVector[0] = targetPoint[0] - rotatedPoint[0];
                lineVector[1] = targetPoint[1] - rotatedPoint[1];
                lineVector[2] = targetPoint[2] - rotatedPoint[2];

                // Calculate the dot product between 2 vectors
                double dotProduct = rotatedPoint[0] * targetPoint[0] + rotatedPoint[1] * targetPoint[1]
                                + rotatedPoint[2] * targetPoint[2];
                double norm1 = Math.sqrt(rotatedPoint[0] * rotatedPoint[0] + rotatedPoint[1] * rotatedPoint[1]
                                + rotatedPoint[2] * rotatedPoint[2]);
                double norm2 = Math.sqrt(targetPoint[0] * targetPoint[0] + targetPoint[1] * targetPoint[1]
                                + targetPoint[2] * targetPoint[2]);
                dotProduct /= (norm1 * norm2);

                // Calculate the rotation axis using cross product.
                double[] rotationAxis = new double[3];
                rotationAxis[0] = rotatedPoint[1] * targetPoint[2] - rotatedPoint[2] * targetPoint[1];
                rotationAxis[1] = rotatedPoint[2] * targetPoint[0] - rotatedPoint[0] * targetPoint[2];
                rotationAxis[2] = rotatedPoint[0] * targetPoint[1] - rotatedPoint[1] * targetPoint[0];
                double rotationAxisNorm = Math.sqrt(rotationAxis[0] * rotationAxis[0]
                                + rotationAxis[1] * rotationAxis[1] + rotationAxis[2] * rotationAxis[2]);
                rotationAxis[0] /= rotationAxisNorm;
                rotationAxis[1] /= rotationAxisNorm;
                rotationAxis[2] /= rotationAxisNorm;

                double angle = Math.acos(dotProduct);

                // Calculate the quaternion parameters using angle and rotation axis
                double halfAngle = angle / 2;
                double sinHalfAngle = Math.sin(halfAngle);

                double[] quaternion = new double[4];
                quaternion[0] = Math.cos(halfAngle);
                quaternion[1] = rotationAxis[0] * sinHalfAngle;
                quaternion[2] = rotationAxis[1] * sinHalfAngle;
                quaternion[3] = rotationAxis[2] * sinHalfAngle;

                double norm = Math.sqrt(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1]
                                + quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
                quaternion[0] /= norm;
                quaternion[1] /= norm;
                quaternion[2] /= norm;
                quaternion[3] /= norm;

                return quaternion;
        }
}
