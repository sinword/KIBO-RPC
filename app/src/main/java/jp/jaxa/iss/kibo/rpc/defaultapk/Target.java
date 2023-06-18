package jp.jaxa.iss.kibo.rpc.defaultapk;

import org.opencv.core.Mat;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcApi;


public class Target {
    private KiboRpcApi api;
    private static int count = 0;

    public Target(KiboRpcApi api) {
        this.api = api;
    }

    public void handleTarget(int targetNumber) {
        api.laserControl(true);
        api.takeTargetSnapshot(targetNumber);
        Mat image = api.getMatNavCam();
        api.saveMatImage(image, "target " + targetNumber + "-" + count + ".png");
        count++;
    }
}
