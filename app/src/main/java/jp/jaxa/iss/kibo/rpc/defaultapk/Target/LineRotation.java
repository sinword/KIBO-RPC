package jp.jaxa.iss.kibo.rpc.defaultapk.Target;

import android.support.compat.R.string;
import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.calib3d.Calib3d;

public class LineRotation {
        private static final String TAG = "LineRotation";

        private static double[] normalize(double[] vector) {
                double norm = Math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
                double[] normalizedVector = new double[3];
                normalizedVector[0] = vector[0] / norm;
                normalizedVector[1] = vector[1] / norm;
                normalizedVector[2] = vector[2] / norm;
                return normalizedVector;
        }

        public static double[] getQuaternion(double[] linePoint, double[] lineDirection, double[] targetPoint) {
                double[] lineVector = new double[3];
                lineVector[0] = targetPoint[0] - linePoint[0];
                lineVector[1] = targetPoint[1] - linePoint[1];
                lineVector[2] = targetPoint[2] - linePoint[2];
                double[] normalizedLineVector = normalize(lineVector);

                double[] noramlizedLineDirection = normalize(lineDirection);

                // Calculate the dot product between noramlizedLineDirection and
                // normalizedLineVector
                double dotProduct = noramlizedLineDirection[0] * normalizedLineVector[0]
                                + noramlizedLineDirection[1] * normalizedLineVector[1]
                                + noramlizedLineDirection[2] * normalizedLineVector[2];

                // Calculate the rotation axis using cross product. normalizedLineVector cross
                // noramlizedLineDirection
                double[] rotationAxis = new double[3];
                rotationAxis[0] = normalizedLineVector[1] * noramlizedLineDirection[2]
                                - normalizedLineVector[2] * noramlizedLineDirection[1];
                rotationAxis[1] = normalizedLineVector[2] * noramlizedLineDirection[0]
                                - normalizedLineVector[0] * noramlizedLineDirection[2];
                rotationAxis[2] = normalizedLineVector[0] * noramlizedLineDirection[1]
                                - normalizedLineVector[1] * noramlizedLineDirection[0];
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

                double norm = Math.sqrt(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1]
                                + quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
                quaternion[0] /= norm;
                quaternion[1] /= norm;
                quaternion[2] /= norm;
                quaternion[3] /= norm;

                return quaternion;
        }
}
