package jp.jaxa.iss.kibo.rpc.defaultapk.Target;

import android.util.Log;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.Point3;

public class Estimation {
        private final String TAG = "Estimation";
        private Mat ids;
        private Mat tvecs;
        // rvecs cannot be used directly, so we need to calculate the rotation matrix
        private Mat rotationMatrix = new Mat();
        private double[][][] rotationMatrixData;

        public Estimation(Mat ids, Mat rvecs, Mat tvecs) {
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
        public Point3 getEstimatedPos() {
                Point3 pos = new Point3(0, 0, 0);
                Log.i(TAG, "In getEstimatedPos");
                Log.i(TAG, "ids: " + ids.dump());
                Log.i(TAG, "tvecs: " + tvecs.dump());

                for (int i = 0; i < tvecs.rows(); i++) {
                        int id = (int) ids.get(i, 0)[0];
                        Log.i(TAG, "id: " + id);
                        double[] data = new double[3];
                        tvecs.get(i, 0, data);
                        Log.i(TAG, "data: " + data[0] + ", " + data[1] + ", " + data[2]);
                        pos.x += data[0];
                        pos.y += data[1];
                        pos.z += data[2];
                        switch (id % 4) {
                                case 1:
                                        Log.i(TAG, "case 1: top right");
                                        // dx = -0.1, dy = 0.0375
                                        pos.x += -0.1f * rotationMatrixData[i][0][0]
                                                        + 0.0375f * rotationMatrixData[i][0][1];
                                        pos.y += -0.1f * rotationMatrixData[i][1][0]
                                                        + 0.0375f * rotationMatrixData[i][1][1];
                                        pos.z += -0.1f * rotationMatrixData[i][2][0]
                                                        + 0.0375f * rotationMatrixData[i][2][1];
                                        break;
                                case 2:
                                        Log.i(TAG, "case 2: top left");
                                        // dx = 0.1, dy = 0.0375
                                        pos.x += 0.1f * rotationMatrixData[i][0][0]
                                                        + 0.0375f * rotationMatrixData[i][0][1];
                                        pos.y += 0.1f * rotationMatrixData[i][1][0]
                                                        + 0.0375f * rotationMatrixData[i][1][1];
                                        pos.z += 0.1f * rotationMatrixData[i][2][0]
                                                        + 0.0375f * rotationMatrixData[i][2][1];
                                        break;
                                case 3:
                                        Log.i(TAG, "case 3: bottom left");
                                        // dx = 0.1, dy = -0.0375
                                        pos.x += 0.1f * rotationMatrixData[i][0][0]
                                                        - 0.0375f * rotationMatrixData[i][0][1];
                                        pos.y += 0.1f * rotationMatrixData[i][1][0]
                                                        - 0.0375f * rotationMatrixData[i][1][1];
                                        pos.z += 0.1f * rotationMatrixData[i][2][0]
                                                        - 0.0375f * rotationMatrixData[i][2][1];
                                        break;
                                case 0:
                                        Log.i(TAG, "case 0: bottom right");
                                        // dx = -0.1, dy = -0.0375
                                        pos.x += -0.1f * rotationMatrixData[i][0][0]
                                                        - 0.0375f * rotationMatrixData[i][0][1];
                                        pos.y += -0.1f * rotationMatrixData[i][1][0]
                                                        - 0.0375f * rotationMatrixData[i][1][1];
                                        pos.z += -0.1f * rotationMatrixData[i][2][0]
                                                        - 0.0375f * rotationMatrixData[i][2][1];
                                        break;
                        }
                }

                pos.x /= tvecs.rows();
                pos.y /= tvecs.rows();
                pos.z /= tvecs.rows();

                return pos;
        }
}
