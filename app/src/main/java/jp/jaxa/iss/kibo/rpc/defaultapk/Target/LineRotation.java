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
                double halfRotaionAngle = Math.acos(rotation[0]);
                double sinHalfRotationAngle = Math.sin(halfRotaionAngle);
                double[] rotationAxis = { rotation[1] / sinHalfRotationAngle, rotation[2] / sinHalfRotationAngle,
                                rotation[3] / sinHalfRotationAngle };
                double[] rotationAxisInWorld = originalVector(rotationAxis, currentOrientation);
                double[] rotationInWorld = { Math.cos(halfRotaionAngle), rotationAxisInWorld[0] * sinHalfRotationAngle,
                                rotationAxisInWorld[1] * sinHalfRotationAngle,
                                rotationAxisInWorld[2] * sinHalfRotationAngle };
                double norm = Math.sqrt(rotationInWorld[0] * rotationInWorld[0]
                                + rotationInWorld[1] * rotationInWorld[1] + rotationInWorld[2] * rotationInWorld[2]
                                + rotationInWorld[3] * rotationInWorld[3]);
                rotationInWorld[0] /= norm;
                rotationInWorld[1] /= norm;
                rotationInWorld[2] /= norm;
                rotationInWorld[3] /= norm;
                return rotationInWorld;
        }

        private static double[] originalVector(double[] convertedVector, double[] quaternion) {
                Mat inputVector = createVector(convertedVector);
                Mat rotationQuaternion = createQuaternion(quaternion);
                Mat inverseRotationMatrix = computeInverseRotationMatrix(rotationQuaternion);
                Mat outputVector = rotateVector(inputVector, inverseRotationMatrix);

                double[] result = new double[outputVector.rows()];
                outputVector.get(0, 0, result);
                result = normalize(result);
                return result;
        }

        private static Mat computeInverseRotationMatrix(Mat rotationQuaternion) {
                Mat rotationMatrix = new Mat();
                Calib3d.Rodrigues(convertQuaternionToRotationVector(rotationQuaternion), rotationMatrix);

                Mat inverseRotationMatrix = new Mat();
                transpose(rotationMatrix, inverseRotationMatrix);

                return inverseRotationMatrix;
        }

        private static Mat createVector(double[] values) {
                Mat vector = new Mat(values.length, 1, CvType.CV_64FC1);
                vector.put(0, 0, values);
                return vector;
        }

        private static Mat createQuaternion(double[] values) {
                Mat quaternion = new Mat(values.length, 1, CvType.CV_64FC1);
                quaternion.put(0, 0, values);
                return quaternion;
        }

        private static Mat rotateVector(Mat inputVector, Mat rotationQuaternion) {
                Mat rotationMatrix = new Mat();
                Calib3d.Rodrigues(convertQuaternionToRotationVector(rotationQuaternion), rotationMatrix);

                Mat outputVector = new Mat();
                gemm(rotationMatrix, inputVector, 1.0, new Mat(), 0.0, outputVector);

                return outputVector;
        }

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

                Mat result = new Mat(3, 1, CvType.CV_64FC1);
                result.put(0, 0, xRot, yRot, zRot);
                return result;
        }
}
