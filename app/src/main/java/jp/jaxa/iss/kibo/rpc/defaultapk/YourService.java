package jp.jaxa.iss.kibo.rpc.defaultapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

/**
 * Class meant to handle commands from the Ground Data System and execute them
 * in Astrobee
 */

public class YourService extends KiboRpcService {

    private static final int LOOP_LIMIT = 5;

    @Override
    protected void runPlan1() {
        // write your plan 1 here
    }

    @Override
    protected void runPlan2() {
        // write your plan 2 here
    }

    @Override
    protected void runPlan3() {
        // write your plan 3 here
    }

    protected void handleQRCode() {

    }

    protected void handleTarget() {
        // avoid KOZ
        Point point = new Point(10.3f, -10.2f, 4.32f);
        Quaternion quaternion = new Quaternion(0f, 0f, 0f, 1f);
        api.moveTo(point, quaternion, true);
        // point 1
        point = new Point(11.2746d, -9.92284d, 5.2988d);
        quaternion = new Quaternion(0.0f, 0.0f, -0.707f, 0.707f);
        api.moveTo(point, quaternion, true);
        api.laserControl(true);
        api.takeTargetSnapshot(1);
        quaternion = new Quaternion(0.707f, 0.0f, 0.0f, 0.707f);
        api.moveTo(point, quaternion, true);
        api.laserControl(true);
        api.takeTargetSnapshot(1);
        api.reportMissionCompletion("Mission Complete!");
    }

}
