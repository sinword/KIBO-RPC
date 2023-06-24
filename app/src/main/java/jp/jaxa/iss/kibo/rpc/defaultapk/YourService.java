package jp.jaxa.iss.kibo.rpc.defaultapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import gov.nasa.arc.astrobee.types.*;
import gov.nasa.arc.astrobee.Result;
import jp.jaxa.iss.kibo.rpc.defaultapk.Target.TargetManager;
import jp.jaxa.iss.kibo.rpc.defaultapk.Target.TargetConfig;
import gov.nasa.arc.astrobee.types.Quaternion;
import android.util.Log;
import PathCaculation.*;
import Basic.*;

import org.opencv.core.Mat;

import Basic.Vector3D;
import PathCaculation.MapConfig;
import PathCaculation.MapManager;

/**
 * Class meant to handle commands from the Ground Data System and execute them
 * in Astrobee
 */

public class YourService extends KiboRpcService {
    private MapConfig mapConfig = new MapConfig();
    private MapManager mapManager = new MapManager(mapConfig, 0.1f);
    private final String TAG = this.getClass().getSimpleName();
    private TargetConfig targetConfig;

    private Point convertFromVector3D(Vector3D vector) {
        return new Point(vector.getX(), vector.getY(), vector.getZ());
    }

    @Override
    protected void runPlan1() {
        Log.i(TAG, "Running plan 1");

        targetConfig = new TargetConfig(api.getNavCamIntrinsics());

        Log.i(TAG, "finish init");

        start();

        moveToByShortestPath(mapConfig.StartPoint, mapConfig.Point1);

        handleTarget(1);

        moveToByShortestPath(mapConfig.Point1, mapConfig.Point2);
        Mat image = api.getMatNavCam();
        api.saveMatImage(image, "Point2");

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
        // moveToByShortestPath(mapConfig.Point1, mapConfig.Point2);

    }
    private void handleActivatedTarget(){
        List<Integer> activatedTargets = getActivatedTargets();
        double[] distances = new double[10];
        for (Integer targetID: activatedTargets) {
            distances[targetID] = getDistanceToPosition(getPointFromID(targetID));
        }
        int minIndex = 0;
        for (int i = 1; i < distances.length; i++) {
            if(distances[i] < distances[minIndex]){
                minIndex = i;
            }
        }
        moveToPointNumber(minIndex);
    }

    private void moveToQRCodePoint(){
        moveToFromCurrentPosition(mapConfig.QRCodePoint);
    }
    private void moveToGoalPoint(){
        moveToFromCurrentPosition(mapConfig.GoalPoint);
    }
    private void moveToPointNumber(int pointNumber){
        Transform des = getPointFromID(pointNumber);
        moveToFromCurrentPosition(des);
    }


    private Transform getPointFromID(int id){
        return mapConfig.AllPoints[id - 1];
    }
    private double getDistanceToPosition(Transform transform){
        Vector3D[] path = mapManager.getShortestPath(Vector3D(getRobotKinematics().getPosition()), transform.getVector3DPosition());
        return mapManager.getPathLength(path);
    }
    private void moveToFromCurrentPosition(Transform to){
        Kinematics kinematics = getRobotKinematics()
        Point point = kinematics.getPosition();
        moveToByShortestPath(point, to);
    }

    private void moveToByShortestPath(Transform transform, Transform to) {
        moveToByShortestPath(transform.position, to);
    }

    private void moveToByShortestPath(Point point, Transform to) {
        Vector3D[] result = mapManager.getShortestPath(new Vector3D(point), to.getVector3DPosition());
        for (int i = 1; i < result.length; i++) {
            directMoveTo(result[i].toPoint(), to.orientation);
        }
    }

    private void directMoveTo(Transform transform) {
        Point point = transform.position;
        Quaternion quaternion = transform.orientation;
        directMoveTo(point, quaternion);
    }

    private void directMoveTo(Point point) {
        Quaternion quaternion = new Quaternion();
        directMoveTo(point, quaternion);
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
