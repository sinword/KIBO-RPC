package jp.jaxa.iss.kibo.rpc.defaultapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import gov.nasa.arc.astrobee.types.*;
import gov.nasa.arc.astrobee.Result;
import jp.jaxa.iss.kibo.rpc.defaultapk.Target.TargetManager;
import jp.jaxa.iss.kibo.rpc.defaultapk.Target.TargetConfig;
import gov.nasa.arc.astrobee.types.Quaternion;
import android.util.Log;
import gov.nasa.arc.astrobee.Kinematics;
import Basic.*;

import org.opencv.core.Mat;

import java.util.List;

import PathCaculation.MapConfig;
import PathCaculation.MapManager;

/**
 * Class meant to handle commands from the Ground Data System and execute them
 * in Astrobee
 */

public class YourService extends KiboRpcService {
    private MapConfig mapConfig = new MapConfig();
    private MapManager mapManager = new MapManager(mapConfig,
            0.1f);private Map<Integer,double>DistanceMap=new HashMap<Integer,double>(); // key: point id, value:
                                                                                        // distance from Astrobee Point
    private boolean QRCodeDown = false;
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

        int count = 0;
        while (api.getActiveTargets().size() != 0 && count < 3) {
            handleActivatedTarget();
            count++;
        }

        moveToQRCodePoint();

        moveToGoalPoint();

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

    private void handleActivatedTarget() {
        List<Integer> activatedTargets = api.getActiveTargets();
        int target = getNextDestination(activatedTargets);
        if (target == 0 && !QRCodeDown) {
            moveToQRCodePoint();
            QRCodeDown = true;
        } else {
            moveToPointNumber(target);
            handleTarget(target);
        }
    }

    private Integer getNextDestination(List<Integer> activatedTargets){
        updateDistanceMap();
        int minDistance = 100000;
        int target = 0;
        if(!QRCodeDown){
            activatedTargets.add(0);
        }
        for (int i = 0; i < activatedTargets.size(); i++) {
            int distance = DistanceMap.get(activatedTargets.get(i));
            if (distance < minDistance){
                minDistance = distance;
                target = activatedTargets.get(i);
            }
        }
        return target;
    }

    private void updateDistanceMap(){
        for (int i = 1; i <= 7; i++) {
            DistanceMap.put(i, getDistanceToPosition(getPointFromID(i)));
        }
        DistanceMap.put(0, getDistanceToPosition(mapConfig.QRCodePoint));
    }

    private void moveToQRCodePoint() {
        moveToFromCurrentPosition(mapConfig.QRCodePoint);
    }

    private void moveToGoalPoint() {
        api.notifyGoingToGoal();
        moveToFromCurrentPosition(mapConfig.GoalPoint);
    }

    private void moveToPointNumber(int pointNumber) {
        Transform des = getPointFromID(pointNumber);
        moveToFromCurrentPosition(des);
    }

    private Transform getPointFromID(int id) {
        return mapConfig.AllPoints[id - 1];
    }

    private double getDistanceToPosition(Transform transform) {
        Vector3D[] path = mapManager.getShortestPath(new Vector3D(api.getRobotKinematics().getPosition()),
                transform.getVector3DPosition());
        return mapManager.getPathLength(path);
    }

    private void moveToFromCurrentPosition(Transform to) {
        Kinematics kinematics = api.getRobotKinematics();
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
