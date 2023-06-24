package jp.jaxa.iss.kibo.rpc.defaultapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import gov.nasa.arc.astrobee.types.*;
import gov.nasa.arc.astrobee.Result;
import android.util.Log;
import PathCaculation.*;
import Basic.*;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.core.Core.gemm;

/**
 * Class meant to handle commands from the Ground Data System and execute them
 * in Astrobee
 */

public class YourService extends KiboRpcService {
    private MapConfig mapConfig = new MapConfig();
    private MapManager mapManager = new MapManager(mapConfig, 0.1f);
    private Map<Integer, double> DistanceMap = new HashMap<Integer, double>(); // key: point id, value: distance from Astrobee Point
    private boolean QRCodeDown = false;
    private final String TAG = this.getClass().getSimpleName();
    private Config config;

    // relative to robot body

    private class Config {
        private final float[] NAV_CAM_POSITION = new float[] { 0.1177f,
                -0.0422f, -0.0826f };
        private final float[] LASER_POSITION = new float[] { 0.1302f, 0.0572f,
                -0.1111f };
        private final float MARKER_LENGTH = 0.05f;
        private int[] count = new int[] { 0, 0, 0, 0, 0, 0, 0 };
        private final int LOOP_LIMIT = 5;
        private MatOfPoint3 objPoints = new MatOfPoint3(
                new Point3(-MARKER_LENGTH / 2.0f, MARKER_LENGTH / 2.0f, 0f),
                new Point3(MARKER_LENGTH / 2.0f, MARKER_LENGTH / 2.0f, 0f),
                new Point3(MARKER_LENGTH / 2.0f, -MARKER_LENGTH / 2.0f, 0f),
                new Point3(-MARKER_LENGTH / 2.0f, -MARKER_LENGTH / 2.0f, 0f));
        private Mat navCamMatrix = new Mat(3, 3, CvType.CV_32FC1);
        private MatOfDouble distCoeffs = new MatOfDouble();
        private Dictionary arucoDict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

        private void init() {
            navCamMatrix.put(0, 0, api.getNavCamIntrinsics()[0]);
            distCoeffs.put(0, 0, api.getNavCamIntrinsics()[1]);
        }
    }

    @Override
    protected void runPlan1() {
        Log.i(TAG, "Running plan 1");

        config = new Config();
        config.init();

        Log.i(TAG, "finish init");

        start();

//        goToPoint1();

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
        //moveToByShortestPath(mapConfig.Point1, mapConfig.Point2);

    }
    private void handleActivatedTarget(){
        List<Integer> activatedTargets = getActivatedTargets();
        int target = getNextDestination(activatedTargets);
        if(target == 0 && !QRCodeDown){
            moveToQRCodePoint();
            QRCodeDown = true;
        }
        else{
            moveToPointNumber(target);
            //handleTarget(target);
        }
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

    private Integer getNextDestination(List<Integer> activatedTargets){
        updateDistanceMap();
        int minDistance = 100000;
        int target = 0;
        if(!QRCodeDown){
            activatedTargets.add(0);
        }
        for (int i = 0; i < activatedTargets.size(); i++) {
            int distance = DistanceMap.get(activatedTargets.[i]);
            if (distance < minDistance){
                minDistance = distance;
                target = activatedTargets[i];
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
    private void moveToByShortestPath(Transform transform, Transform to){
        moveToByShortestPath(transform.position, to);
    }
    private void moveToByShortestPath(Point point, Transform to){
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
    private void directMoveTo(Point point, Quaternion quaternion)
    {
        Result result;
        int count = 0, max_count = 3;
        do
        {
            result = api.moveTo(point, quaternion, true);
            count++;
        }
        while (!result.hasSucceeded() && count < max_count);
    }

//    private void goToPoint1() {
//        // avoid KOZ
//        MapManager manager = new MapManager(new MapConfig(), 1);
//        var source = config.StartPoint;
//        var destination = config.Point1;
//        var path = manager.getShortestPath(source, destination);
//        Quateration quaternion = new Quateration(0f, 0f, 0f, 1f);
//        for (Point point: path) {
//            api.moveTo(point, quaternion, true);
//        }
////        Point point = new Point(10.3f, -10.2f, 4.32f);
////        Quaternion quaternion = new Quaternion(0f, 0f, 0f, 1f);
//
//        // point 1
//        point = new Point(11.2746d, -9.92284d, 5.2988d);
//        quaternion = new Quaternion(0.0f, 0.0f, -0.707f, 0.707f);
//        api.moveTo(point, quaternion, true);
//    }

    private void start() {
        Log.i(TAG, "start mission");
        // the mission starts
        api.startMission();
    }

    private void calibrateLocation(int targetNumber) {
        Log.i(TAG, "In calibrateLocation");
        Mat image = api.getMatNavCam();
        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();
        Aruco.detectMarkers(image, config.arucoDict, corners, ids);
        Aruco.drawDetectedMarkers(image, corners, ids);

        if (ids.size().empty()) {
            return;
        }

        Mat rvecs = new Mat();
        Mat tvecs = new Mat();
        Log.i(TAG, "Estimating markers pose");
        Aruco.estimatePoseSingleMarkers(corners, config.MARKER_LENGTH, config.navCamMatrix,
                config.distCoeffs, rvecs, tvecs);

        // Iterate over each marker and calculate its center point
        for (int i = 0; i < ids.size().height; ++i) {
            Mat rvec = rvecs.row(i);
            Mat tvec = tvecs.row(i);

            // Draw marker axes on the image
            Calib3d.drawFrameAxes(image, config.navCamMatrix, config.distCoeffs, rvec, tvec,
                    config.MARKER_LENGTH / 2.0f);

            // Convert rotation vector to rotation matrix
            Mat rotMat = new Mat();
            Calib3d.Rodrigues(rvec, rotMat);

            Log.d(TAG, "rotMat.inv(): " + rotMat.inv().dump());
            Log.d(TAG, "tvec.t(): " + tvec.t().dump());
            Log.d(TAG, "type of rotMat.inv(): " + rotMat.inv().type());
            Log.d(TAG, "type of tvec.t(): " + tvec.t().type());
            Log.d(TAG, "size of rotMat.inv(): " + rotMat.inv().size());
            Log.d(TAG, "size of tvec.t(): " + tvec.t().size());

            // Calculate marker center point
            // Mat markerPoint = new Mat();
            // gemm(rotMat.inv(), tvec.t(), 1, new Mat(), 0, markerPoint);

            // markerPoint = markerPoint.t();

            // // Log center point of marker
            // Log.i(TAG, "Marker " + ids.get(i, 0)[0] + " center point: "
            // + markerPoint.dump());
        }

        api.saveMatImage(image, "target_" + targetNumber + "_" + config.count[targetNumber]
                + ".png");
        config.count[targetNumber]++;
    }

    private void handleTarget(int targetNumber) {
        calibrateLocation(targetNumber);
        api.laserControl(true);
        api.takeTargetSnapshot(targetNumber);
        Mat image = api.getMatNavCam();
        api.saveMatImage(image, "target_" + targetNumber + "_laser.png");
    }
}
