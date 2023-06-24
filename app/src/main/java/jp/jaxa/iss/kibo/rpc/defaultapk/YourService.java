package jp.jaxa.iss.kibo.rpc.testapk;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.api.types.PointCloud;
// astrobee library
import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;
// android library
import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;
// zxing library
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;
import static org.opencv.android.Utils.matToBitmap;
// opencv library
import java.util.ArrayList;
import java.util.List;
// java library

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    int max_count = 5, center_range = 6, P1 = 0, P2 = 1;
    private final String TAG = this.getClass().getSimpleName();

    @Override
    protected void runPlan1(){
        Log.i(TAG, "start mission");

        // mission starts
        api.startMission();

        int loop_counter = 0;
        int max_count = 3;


        Point point = new Point(11.381944f, -8.566172f, 4.32f);
        Quaternion quaternion = new Quaternion(0, 0, 0, 1);
        api.moveTo(point, quaternion, true);


        String report_massage = HandleQRCode();
        Log.i(TAG, "QR code report message" + report_massage);
    }

    @Override
    protected void runPlan2(){
        // write your plan 2 here
    }

    @Override
    protected void runPlan3(){
        // write your plan 3 here
    }

    private String HandleQRCode() {
        String contents = null;
        int count = 0;
        int count_max = 5;
        String report_message = "";

        while(contents == null && count < count_max) {
            Log.i(TAG, "QRcode event start!");
            long start_time = SystemClock.elapsedRealtime();

            // move to a point
//            Point point = new Point(px, py, pz);
//            Quaternion quaternion = new Quaternion(qx, qy, qz, qw);
//            api.moveTo(point, quaternion, true);

            // turn on the front flash light
            flash_control(true);
            // scan QRcode
            // Mat qr_mat = new Mat(undistord(api.getMatNavCam()), cropImage(40)); //裁切大小?
            Mat qr_mat = api.getMatNavCam();
            Bitmap bMap = resizeImage(qr_mat, 2000, 1500);	// 大小重設?


            //////////////////////////////////////////////////////////////////////////////////////////////////////
            int[] intArray = new int[bMap.getWidth() * bMap.getHeight()];
            bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());

            LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArray);
            BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));


            try{
                com.google.zxing.Result result = new QRCodeReader().decode(bitmap);
                contents = result.getText();	// 得到掃到的資料
                Log.i(TAG, "QR code is detected, content is " + contents);

                switch (contents) {
                    case "JEM":
                        report_message = "STAY_AT_JEM";
                        break;
                    case "COLUMBUS":
                        report_message = "GO_TO_COLUMBUS";
                        break;
                    case "RACK1":
                        report_message = "CHECK_RACK_1";
                        break;
                    case "ASTROBEE":
                        report_message = "I_AM_HERE";
                        break;
                    case "INTBALL":
                        report_message = "LOOKING_FORWARD_TO_SEE_YOU";
                        break;
                    case "BLANK":
                        report_message = "NO_PROBLEM";
                        break;
                    default:
                        report_message = null;
                        break;
                }

            }
            catch (Exception e) {
                Log.i(TAG, "QR code is not detected");
            }
            //////////////////////////////////////////////////////////////////////////////////////////////////////
            Log.i(TAG, "QRcode event stop");
            long stop_time = SystemClock.elapsedRealtime();

            Log.i(TAG, "QRcode event spent time: "+ (stop_time-start_time)/1000);
            count++;
        }
        flash_control(false);

        // report mission completion and blink the lights
        api.reportMissionCompletion(report_message);
        return report_message;
    }

    private void flash_control(boolean status) { // 新增 thread 一秒等待 flash打開
        if(status) {
            api.flashlightControlFront(0.05f); //1st code是給 0.025f
            api.flashlightControlBack(0.025f);	// 1st code 沒給
            try {
                Thread.sleep(1000); // wait a few seconds
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        else api.flashlightControlFront(0.00f);
    }

//    private Mat undistord(Mat src)
//    {
//        Mat dst = new Mat(1280, 960, CvType.CV_8UC1);
//        Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
//        Mat distCoeffs = new Mat(1, 5, CvType.CV_64F);
//
//        int row = 0, col = 0;
//
//        double NavCam[][] = api.getNavCamIntrinsics();
//        double NavCam_cameraMatrix[][] = new double[3][3];
//        double NavCam_distCoeffs[] = new double[5];
//
//        double DockCam[][] = api.getDockCamIntrinsics();
//        double DockCam_cameraMatrix[][] = new double[3][3];
//        double DockCam_distCoeffs[] = new double[5];
//
//        int counter = 0;
//        for(int i=0; i<3; i++){
//            for(int j=0; j<3; j++){
//                NavCam_cameraMatrix[i][j] = NavCam[0][counter];
//                DockCam_cameraMatrix[i][j] = DockCam[0][counter];
//                counter++;
//            }
//        }
//        for(int i=0; i<5; i++){
//            NavCam_distCoeffs[i] = NavCam[1][i];
//            DockCam_distCoeffs[i] = DockCam[1][i];
//        }
//        /*double cameraMatrix_sim[] = {
//                344.173397, 0.000000, 630.793795,
//                0.000000, 344.277922, 487.033834,
//                0.000000, 0.000000, 1.000000
//        };
//		double distCoeffs_sim[] = {-0.152963, 0.017530, -0.001107, -0.000210, 0.000000};
//        double cameraMatrix_orbit[] = {
//                692.827528, 0.000000, 571.399891,
//                0.000000, 691.919547, 504.956891,
//                0.000000, 0.000000, 1.000000
//        };
//
//        double distCoeffs_orbit[] = {-0.312191, 0.073843, -0.000918, 0.001890, 0.000000};
//		*/
//
//        if(MODE == "sim") {
//            cameraMatrix.put(row, col, NavCam_cameraMatrix);
//            distCoeffs.put(row, col, NavCam_distCoeffs);
//            Log.d("Mode[camera]:"," sim");
//        }
//        else if(MODE == "iss") {
//            cameraMatrix.put(row, col, DockCam_cameraMatrix);
//            distCoeffs.put(row, col, DockCam_distCoeffs);
//            Log.d("Mode[camera]:"," iss");
//        }
//
//        cameraMatrix.put(row, col, NavCam_cameraMatrix);
//        distCoeffs.put(row, col, NavCam_distCoeffs);
//
//        Imgproc.undistort(src, dst, cameraMatrix, distCoeffs);
//        return dst;
//    }

    private Bitmap resizeImage(Mat src, int width, int height) {
        Size size = new Size(width, height);
        Imgproc.resize(src, src, size);

        Bitmap bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        matToBitmap(src, bitmap, false);
        return bitmap;
    }
}
