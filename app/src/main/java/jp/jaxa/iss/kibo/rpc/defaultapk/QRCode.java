package jp.jaxa.iss.kibo.rpc.defaultapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcApi;

public class QRCode {
    private KiboRpcApi api;

    public QRCode(KiboRpcApi api) {
        this.api = api;
    }

    public void handleQRCode() {
        api.getActiveTargets();
    }
}
