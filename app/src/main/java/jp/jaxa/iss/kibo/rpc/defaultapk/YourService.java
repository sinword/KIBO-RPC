package jp.jaxa.iss.kibo.rpc.defaultapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.defaultapk.Target.TargetManager;
import jp.jaxa.iss.kibo.rpc.defaultapk.Target.TargetConfig;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import android.util.Log;

import org.opencv.core.Mat;

import Basic.Vector3D;
import PathCaculation.MapConfig;
import PathCaculation.MapManager;

/**
 * Class meant to handle commands from the Ground Data System and execute them
 * in Astrobee
 */

public class YourService extends KiboRpcService {
    private final String TAG = this.getClass().getSimpleName();
    private TargetConfig targetConfig;
    private MapManager mapManager;

    private Point convertFromVector3D(Vector3D vector) {
        return new Point(vector.getX(), vector.getY(), vector.getZ());
    }

    @Override
    protected void runPlan1() {
        Log.i(TAG, "Running plan 1");

        targetConfig = new TargetConfig(api.getNavCamIntrinsics());
        MapConfig mapConfig = new MapConfig();
        mapManager = new MapManager(mapConfig, 0.1);
        Vector3D from = mapConfig.StartPoint;
        Vector3D to = mapConfig.Point1;
        Vector3D[] result = mapManager.getShortestPath(from, to);

        Log.i(TAG, "finish init");

        start();

        Quaternion currentOrientation = api.getRobotKinematics().getOrientation();
        for (Vector3D point : result) {
            Log.i(TAG, "point: " + point.toString());
            Point pointInPoint = convertFromVector3D(point);
            api.moveTo(pointInPoint, currentOrientation, true);
        }

        handleTarget(1);

        api.reportMissionCompletion("Mission Complete!");
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

    private void start() {
        Log.i(TAG, "start mission");
        // the mission starts
        api.startMission();
    }

    private void handleTarget(int targetNumber) {
        Mat image = api.getMatNavCam();
        Quaternion absoluteOrientation = TargetManager.calibrateLocation(image,
                api.getRobotKinematics().getOrientation());
        api.saveMatImage(image, "target_" + targetNumber + "_" + targetConfig.count[targetNumber]
                + ".png");
        TargetConfig.count[targetNumber]++;
        api.relativeMoveTo(new Point(0, 0, 0), absoluteOrientation, true);
        api.laserControl(true);
        api.takeTargetSnapshot(targetNumber);
    }
}
