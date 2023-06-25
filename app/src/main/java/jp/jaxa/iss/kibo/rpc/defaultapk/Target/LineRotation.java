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
                // Extract the rotation angle and axis from the quaternion
                double halfRotationAngle = Math.acos(rotation[0]);
                double sinHalfRotationAngle = Math.sin(halfRotationAngle);
                double[] rotationAxis = { rotation[1], rotation[2], rotation[3] };
                rotationAxis = normalize(rotationAxis);

                // Calculate the original rotation axis in the world coordinate system
                double[] rotationAxisInWorld = calculateOriginalRotationAxis(rotationAxis, currentOrientation);

                // Convert the rotation to the real-world representation
                double[] rotationInWorld = { Math.cos(halfRotationAngle), rotationAxisInWorld[0] * sinHalfRotationAngle,
                                rotationAxisInWorld[1] * sinHalfRotationAngle,
                                rotationAxisInWorld[2] * sinHalfRotationAngle };
                rotationInWorld = normalize(rotationInWorld);
                return rotationInWorld;
        }

        private static double[] calculateOriginalRotationAxis(double[] inputRotationAxis, double[] robotOrientation) {
                Mat inputRotationAxisVector = createVector(inputRotationAxis);
                Mat robotOrientationQuaternion = createQuaternion(robotOrientation);
                Mat inverseRotationMatrix = computeInverseRotationMatrix(robotOrientationQuaternion);
                Mat outputRotationAxisVector = rotateVector(inputRotationAxisVector, inverseRotationMatrix);
                double[] outputVector = convertVectorToArray(outputRotationAxisVector);
                return normalize(outputVector);
        }

        // Create a 3D vector (column matrix) from an array of values
        private static Mat createVector(double[] values) {
                Mat vector = new Mat(values.length, 1, CvType.CV_64FC1);
                vector.put(0, 0, values);
                return vector;
        }

        // Create a quaternion (column matrix) from an array of values
        private static Mat createQuaternion(double[] values) {
                Mat quaternion = new Mat(values.length, 1, CvType.CV_64FC1);
                quaternion.put(0, 0, values);
                return quaternion;
        }

        // Compute the inverse rotation matrix from a quaternion
        private static Mat computeInverseRotationMatrix(Mat quaternion) {
                Mat rotationMatrix = new Mat();
                Calib3d.Rodrigues(convertQuaternionToRotationVector(quaternion), rotationMatrix);

                Mat inverseRotationMatrix = new Mat();
                transpose(rotationMatrix, inverseRotationMatrix);

                return inverseRotationMatrix;
        }

        // Rotate a 3D vector using a rotation matrix
        private static Mat rotateVector(Mat inputVector, Mat rotationMatrix) {
                Mat outputVector = new Mat();
                gemm(rotationMatrix, inputVector, 1.0, new Mat(), 0.0, outputVector);

                return outputVector;
        }

        // Convert a quaternion to a rotation vector
        private static Mat convertQuaternionToRotationVector(Mat quaternion) {
                double w = quaternion.get(0, 0)[0];
                double x = quaternion.get(1, 0)[0];
                double y = quaternion.get(2, 0)[0];
                double z = quaternion.get(3, 0)[0];

                double angle = 2 * Math.acos(w);
                double sinAngle = Math.sin(angle / 2);

                double xRot = x / sinAngle * angle;
                double yRot = y / sinAngle * angle;
                double zRot = z / sinAngle * angle;

                Mat rotationVector = new Mat(3, 1, CvType.CV_64FC1);
                rotationVector.put(0, 0, xRot, yRot, zRot);

                return rotationVector;
        }

        // Convert a 3D vector to an array of values
        private static double[] convertVectorToArray(Mat vector) {
                double[] values = new double[vector.rows()];
                for (int i = 0; i < vector.rows(); i++) {
                        values[i] = vector.get(i, 0)[0];
                }
                return values;
        }
}
