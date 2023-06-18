package jp.jaxa.iss.kibo.rpc.defaultapk;

import org.opencv.core.Mat;

public class Target extends YourService {
    private static int count = 0;

    public void handleTarget(int targetNumber) {
        api.laserControl(true);
        api.takeTargetSnapshot(targetNumber);
        Mat image = api.getMatNavCam();
        api.saveMatImage(image, "target " + targetNumber + "-" + count + ".png");
        count++;
    }
}
