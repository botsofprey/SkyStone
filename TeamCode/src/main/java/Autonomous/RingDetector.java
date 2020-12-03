package Autonomous;

import android.util.Log;

import SensorHandlers.LIDARSensor;

public class RingDetector extends Thread {

    private static final double BOTTOM_RING_TOLERANCE = 8;
    private static final double TOP_RING_TOLERANCE = 20;

    private volatile boolean topDetected, bottomDetected;
    private volatile boolean shouldRun;
    private final LIDARSensor top, bottom;

    public RingDetector(final LIDARSensor top, final LIDARSensor bottom) {
        this.top = top;
        this.bottom = bottom;
        topDetected = false;
        bottomDetected = false;
        shouldRun = true;

        new Thread(new Runnable() {
            @Override
            public void run() {
                while(shouldRun) {
                    detectNumRings();
                    Log.d("Distance:", "" + top.getDistance() + ", " + bottom.getDistance());
                }
            }
        }).start();
    }

    public int getNumRings() {
        shouldRun = false;
        if (topDetected && bottomDetected) return 4;
        if (bottomDetected) return 1;
        return 0;
    }

    public void detectNumRings() {
        if (top.getDistance() < TOP_RING_TOLERANCE) topDetected = true;
        if (bottom.getDistance() < BOTTOM_RING_TOLERANCE) bottomDetected = true;
    }

}
