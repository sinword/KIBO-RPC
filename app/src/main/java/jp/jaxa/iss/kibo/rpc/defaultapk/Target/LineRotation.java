package jp.jaxa.iss.kibo.rpc.defaultapk.Target;

import android.support.compat.R.string;
import android.util.Log;
import jp.jaxa.iss.kibo.rpc.defaultapk.Target.TargetConfig;
import jp.jaxa.iss.kibo.rpc.defaultapk.Target.TargetManager;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.lang.annotation.Target;

import org.opencv.calib3d.Calib3d;

import static org.opencv.core.Core.gemm;
import static org.opencv.core.Core.invert;
import static org.opencv.core.Core.transpose;

public class LineRotation {
        private static final String TAG = "LineRotation";

        public static double[] normalize(double[] vector) {
                double normSquare = 0;
                for (int i = 0; i < vector.length; i++) {
                        normSquare += vector[i] * vector[i];
                }
                double[] normalizedVector = new double[vector.length];
                double norm = Math.sqrt(normSquare);
                for (int i = 0; i < vector.length; i++) {
                        normalizedVector[i] = vector[i] / norm;
                }
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

        public static double[] convertToRealWorldRotation(double[] rotation, double[] currentOrientation) {
                Mat rotationMatrix = quaternionToRotationMatrix(currentOrientation);

                Mat rotationAxis = transformVector(rotationMatrix, rotation);
                double[] rotationAxisInDouble = new double[3];
                rotationAxisInDouble[0] = rotationAxis.get(0, 0)[0];
                rotationAxisInDouble[1] = rotationAxis.get(1, 0)[0];
                rotationAxisInDouble[2] = rotationAxis.get(2, 0)[0];
                rotationAxisInDouble = normalize(rotationAxisInDouble);
                double halfAngle = Math.acos(rotation[0]);
                double sinHalfAngle = Math.sin(halfAngle);
                double[] realWorldRotation = new double[4];
                realWorldRotation[0] = Math.cos(halfAngle);
                realWorldRotation[1] = -rotationAxisInDouble[0] * sinHalfAngle;
                realWorldRotation[2] = -rotationAxisInDouble[1] * sinHalfAngle;
                realWorldRotation[3] = -rotationAxisInDouble[2] * sinHalfAngle;
                realWorldRotation = normalize(realWorldRotation);
                return realWorldRotation;
        }

        public static Mat quaternionToRotationMatrix(double[] quaternion) {
                double q0 = quaternion[0];
                double q1 = quaternion[1];
                double q2 = quaternion[2];
                double q3 = quaternion[3];

                double r00 = 2 * (q0 * q0 + q1 * q1) - 1;
                double r01 = 2 * (q1 * q2 - q0 * q3);
                double r02 = 2 * (q1 * q3 + q0 * q2);
                double r10 = 2 * (q1 * q2 + q0 * q3);
                double r11 = 2 * (q0 * q0 + q2 * q2) - 1;
                double r12 = 2 * (q2 * q3 - q0 * q1);
                double r20 = 2 * (q1 * q3 - q0 * q2);
                double r21 = 2 * (q2 * q3 + q0 * q1);
                double r22 = 2 * (q0 * q0 + q3 * q3) - 1;

                Mat rotationMatrix = new Mat(3, 3, CvType.CV_64F);
                rotationMatrix.put(0, 0, r00);
                rotationMatrix.put(0, 1, r01);
                rotationMatrix.put(0, 2, r02);
                rotationMatrix.put(1, 0, r10);
                rotationMatrix.put(1, 1, r11);
                rotationMatrix.put(1, 2, r12);
                rotationMatrix.put(2, 0, r20);
                rotationMatrix.put(2, 1, r21);
                rotationMatrix.put(2, 2, r22);

                return rotationMatrix;
        }

        public static Mat transformVector(Mat rotationMatrix, double[] vector) {
                Mat vectorInRotatedSystem = new Mat(3, 1, CvType.CV_64FC1);
                vectorInRotatedSystem.put(0, 0, vector[1]);
                vectorInRotatedSystem.put(1, 0, vector[2]);
                vectorInRotatedSystem.put(2, 0, vector[3]);

                Mat inverseRotationMatrix = new Mat();
                invert(rotationMatrix, inverseRotationMatrix);

                Mat vectorInOriginalSystem = new Mat();
                gemm(inverseRotationMatrix, vectorInRotatedSystem, 1, new Mat(), 0, vectorInOriginalSystem);

                return vectorInOriginalSystem;
        }
}
