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

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import PathCaculation.MapConfig;
import PathCaculation.MapManager;

/**
 * Class meant to handle commands from the Ground Data System and execute them
 * in Astrobee
 */

public class YourService extends KiboRpcService {
    private MapConfig mapConfig = new MapConfig();
    private MapManager mapManager = new MapManager(mapConfig, 0.12f);
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
            handleNextDestination();
            count++;
        }

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

    private void handleNextDestination() {
        List<Integer> activatedTargets = api.getActiveTargets();
        if (!QRCodeDown) {
            activatedTargets.add(0);
        }
        int target = getNextDestination(activatedTargets);
        if (target == 0) {
            moveToQRCodePoint();
            QRCodeDown = true;
        } else {
            moveToPointNumber(target);
            handleTarget(target);
        }
    }

    private void moveToPointNumber(int pointNumber) {
        Transform des = getPointFromID(pointNumber);
        moveToFromCurrentPosition(des);
    }

    private Integer getNextDestination(List<Integer> activatedTargets) {
        Map<Integer, Double> DistanceMap = getDistanceMap();
        double minDistance = 100000;
        int target = -1;
        for (int i = 0; i < activatedTargets.size(); i++) {
            double distance = DistanceMap.get(activatedTargets.get(i));
            if (distance < minDistance) {
                minDistance = distance;
                target = activatedTargets.get(i);
            }
        }
        return target;
    }

    private Map<Integer, Double> getDistanceMap() {
        Map<Integer, Double> DistanceMap = new HashMap<Integer, Double>();
        for (int i = 1; i <= 7; i++) {
            DistanceMap.put(i, getDistanceToPosition(getPointFromID(i)));
        }
        DistanceMap.put(0, getDistanceToPosition(mapConfig.QRCodePoint));
        return DistanceMap;
    }

    private Transform getPointFromID(int id) {
        return mapConfig.AllPoints[id - 1];
    }

    private void moveToQRCodePoint() {
        moveToFromCurrentPosition(mapConfig.QRCodePoint);
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
            double[] vector = TargetManager.calibrateLocation(image, newOrientation);
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
}
