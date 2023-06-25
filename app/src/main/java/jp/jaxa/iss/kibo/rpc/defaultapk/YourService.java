package jp.jaxa.iss.kibo.rpc.defaultapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import gov.nasa.arc.astrobee.types.*;
import gov.nasa.arc.astrobee.Result;
import jp.jaxa.iss.kibo.rpc.defaultapk.Target.TargetManager;
import jp.jaxa.iss.kibo.rpc.defaultapk.Target.TargetConfig;
import gov.nasa.arc.astrobee.types.Quaternion;
import gov.nasa.arc.astrobee.Kinematics;
import jp.jaxa.iss.kibo.rpc.api.types.PointCloud;

import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;
import gov.nasa.arc.astrobee.Kinematics;
import Basic.*;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import static org.opencv.android.Utils.matToBitmap;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.opencv.android.Utils.matToBitmap;
import PathCaculation.MapConfig;
import PathCaculation.MapManager;

/**
 * Class meant to handle commands from the Ground Data System and execute them
 * in Astrobee
 */

public class YourService extends KiboRpcService {
    private MapConfig mapConfig = new MapConfig();
    private MapManager mapManager = new MapManager(mapConfig, 0.12f);
    private boolean QRCodeDone = false;
    private final String TAG = this.getClass().getSimpleName();
    private TargetConfig targetConfig;
    // private Map<String, String> QRCodeResultMap = new HashMap<String, String>();
    private String QRCodeResult = "";

    private Point convertFromVector3D(Vector3D vector) {
        return new Point(vector.getX(), vector.getY(), vector.getZ());
    }

    @Override
    protected void runPlan1() {

        Log.i(TAG, "Running plan 1");

        targetConfig = new TargetConfig(api.getNavCamIntrinsics());

        Log.i(TAG, "finish init");

        start();
        MainRun();
    }

    @Override
    protected void runPlan2() {
        Log.i(TAG, "Running plan 2");
        runPlan1();
    }

    @Override
    protected void runPlan3() {
        Log.i(TAG, "Running plan 3");
        runPlan1();
    }

    private void MainRun() {
        while (true) {
            if( api.getTimeRemaining().get(1) < 60000){
                moveToFromCurrentPosition(mapConfig.GoalPoint);
                api.notifyGoingToGoal();
                handleGoal();
                break;
            }
            Integer point = moveToShortestAvailablePoint();
            Log.i(TAG, "[MainRun] move to point: " + point);
            if (point == 0) {
                QRCodeDone = true;
                QRCodeResult = HandleQRCode();
            } else if (point == 8) {
                api.notifyGoingToGoal();
                handleGoal();
                break;
            } else if (point >= 1 && point <= 7) {
                handleTarget(point);
            }

            if (getRemainingDistance() < 0.1) {
                break;
            }
        }
    }

    private void handleGoal() {
        if (!QRCodeDone) {
            Log.i(TAG, "No QRCode detected");
        }
        api.reportMissionCompletion(QRCodeResult);
    }

    private Integer moveToShortestAvailablePoint() {
        List<Integer> activeTarget = api.getActiveTargets();
        if (!QRCodeDone) {
            activeTarget.add(0);
        }
        Vector3D currentPosition = new Vector3D(api.getRobotKinematics().getPosition());
        Integer shortestPoint = mapManager.getShortestAvailablePointID(currentPosition, activeTarget);
        moveToFromCurrentPosition(getPointFromID(shortestPoint));
        return shortestPoint;
    }

    private void moveToPointNumber(int pointNumber) {
        Transform des = getPointFromID(pointNumber);
        moveToFromCurrentPosition(des);
    }

    private Double getRemainingDistance() {
        List<Long> timeList = api.getTimeRemaining();
        return time2Distance(timeList.get(1));
    }

    private Double getActiveTargetRemainingDistance() {
        List<Long> timeList = api.getTimeRemaining();
        return time2Distance(timeList.get(0));
    }

    private Double time2Distance(Long millisecond) {
        double Velocity = 0.5; // m/s
        return Velocity * millisecond / 1000;
    }

    private Transform getPointFromID(int id) {
        return mapConfig.getTransformMap().get(id);
    }

    private void moveToFromCurrentPosition(Transform to) {
        Kinematics kinematics = api.getRobotKinematics();
        Point point = kinematics.getPosition();
        moveToByShortestPath(point, to);
    }

    private void moveToByShortestPath(Point point, Transform to) {
        Vector3D[] result = mapManager.getShortestPath(new Vector3D(point), to.getVector3DPosition());
        for (int i = 1; i < result.length; i++) {
            directMoveTo(result[i].toPoint(), to.orientation);
        }
    }

    private void directMoveTo(Point point, Quaternion quaternion) {
        Log.i(TAG, "move to " + point.toString());
        Result result;
        int count = 0, max_count = 3;
        do {
            result = api.moveTo(point, quaternion, true);
            count++;
        } while (!result.hasSucceeded() && count < max_count);
    }

    private void start() {
        Log.i(TAG, "start mission");
        // the mission starts
        api.startMission();
    }

    private void handleTarget(int targetNumber) {
        int loopCount = 0;
        Quaternion original = api.getRobotKinematics().getOrientation();
        float[] closestValues = { snapToValues(original.getX()), snapToValues(original.getY()),
                snapToValues(original.getZ()), snapToValues(original.getW()) };
        Quaternion newOrientation = new Quaternion(closestValues[0], closestValues[1], closestValues[2],
                closestValues[3]);
        double movement = Integer.MAX_VALUE;
        while (movement > 0.03 && loopCount < 3) {
            Mat image = api.getMatNavCam();
            double[] vector = TargetManager.calibrateLocation(image, newOrientation, targetNumber);
            double smallest = Math.abs(vector[0]);
            int id = 0;
            for (int i = 1; i < vector.length; i++) {
                if (Math.abs(vector[i]) < smallest) {
                    smallest = Math.abs(vector[i]);
                    id = i;
                }
            }
            vector[id] = 0;
            api.relativeMoveTo(new Point(vector[0], vector[1], vector[2]), newOrientation, true);
            movement = Math.sqrt(Math.pow(vector[0], 2) + Math.pow(vector[1], 2) + Math.pow(vector[2], 2));
            loopCount++;
        }

        api.laserControl(true);
        api.takeTargetSnapshot(targetNumber);
    }

    public static float snapToValues(float input) {
        float[] values = { -1, -0.707f, -0.5f, 0, 0.5f, 0.707f, 1 };
        float closestValue = values[0];
        float smallestDifference = Math.abs(input - closestValue);

        for (int i = 1; i < values.length; i++) {
            float difference = Math.abs(input - values[i]);
            if (difference < smallestDifference) {
                closestValue = values[i];
                smallestDifference = difference;
            }
        }

        return closestValue;
    }

    private Bitmap resizeImage(Mat src, int width, int height) {
        Size size = new Size(width, height);
        Imgproc.resize(src, src, size);

        Bitmap bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        matToBitmap(src, bitmap, false);
        return bitmap;
    }

    private String HandleQRCode() {
        String contents = null;
        int count = 0;
        int count_max = 5;
        String report_message = "";

        while (contents == null && count < count_max) {
            Log.i(TAG, "QRcode event start!");
            long start_time = SystemClock.elapsedRealtime();
            // turn on the front flash light
            flash_control(true);
            // scan QRcode
            Mat qr_mat = api.getMatNavCam();
            Bitmap bMap = resizeImage(qr_mat, 2000, 1500);

            int[] intArray = new int[bMap.getWidth() * bMap.getHeight()];
            bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());
            LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArray);
            BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));

            try {
                com.google.zxing.Result result = new QRCodeReader().decode(bitmap);
                contents = result.getText(); // 得到掃到的資料
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
            } catch (Exception e) {
                Log.i(TAG, "QR code is not detected");
            }

            Log.i(TAG, "QRcode event stop");
            long stop_time = SystemClock.elapsedRealtime();

            Log.i(TAG, "QRcode event spent time: " + (stop_time - start_time) / 1000);
            count++;
        }
        flash_control(false);

        // report mission completion and blink the lights
        return report_message;
    }

    private void flash_control(boolean status) { // 新增 thread 一秒等待 flash打開
        if (status) {
            api.flashlightControlFront(0.05f); // 1st code是給 0.025f
            api.flashlightControlBack(0.025f); // 1st code 沒給
            try {
                Thread.sleep(1000); // wait a few seconds
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else
            api.flashlightControlFront(0.00f);
    }
}
