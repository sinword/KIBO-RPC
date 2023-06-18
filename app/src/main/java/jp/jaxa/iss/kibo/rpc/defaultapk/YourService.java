package jp.jaxa.iss.kibo.rpc.defaultapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import android.util.Log;

/**
 * Class meant to handle commands from the Ground Data System and execute them
 * in Astrobee
 */

public class YourService extends KiboRpcService {
    private QRCode qrCode;
    private Target target;

    private static final int LOOP_LIMIT = 5;
    private final String TAG = this.getClass().getSimpleName();

    public YourService() {
        super();
        qrCode = new QRCode();
        target = new Target();
    }

    private void goToPoint1() {
        // avoid KOZ
        Point point = new Point(10.3f, -10.2f, 4.32f);
        Quaternion quaternion = new Quaternion(0f, 0f, 0f, 1f);
        api.moveTo(point, quaternion, true);
        // point 1
        point = new Point(11.2746d, -9.92284d, 5.2988d);
        quaternion = new Quaternion(0.0f, 0.0f, -0.707f, 0.707f);
        api.moveTo(point, quaternion, true);
    }

    private void turnToTarget() {
        Point point = new Point(11.2746d, -9.92284d, 5.2988d);
        Quaternion quaternion = new Quaternion(0.707f, 0.0f, 0.0f, 0.707f);
        api.moveTo(point, quaternion, true);
    }

    private void start() {
        Log.i(TAG, "start mission");
        // the mission starts
        api.startMission();
    }

    @Override
    protected void runPlan1() {
        start();

        goToPoint1();

        target.handleTarget(1);

        turnToTarget();

        target.handleTarget(1);

        api.reportMissionCompletion("Mission Complete!");
    }

    @Override
    protected void runPlan2() {
        // write your plan 2 here
    }

    @Override
    protected void runPlan3() {
        // write your plan 3 here
    }
}
