package Actions.Ultimate;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class WheelMotor {

    private boolean debug;

    public DcMotor motor;
    private double rpm;
    private double targetRPM;
    private long prevTicks;
    private long prevTime;
    private static final int MAX_RPM = 6000;
    private static final double MINIMUM_TIME_DIFFERENCE = 100000000;
    private static final double ADJUSTMENT_RATE = 2;

    public WheelMotor(String name, HardwareMap hardwareMap, boolean debug) {
        motor = hardwareMap.dcMotor.get(name);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetRPM = 0;
        this.debug = debug;
    }

    public void setRPM(int RPM) {
        motor.setPower(RPM / (double) MAX_RPM);
        targetRPM = RPM;
    }

    public void updateShooterRPM(OpMode mode) {
        int currentTicks = motor.getCurrentPosition();
        long currentTime = System.nanoTime();
        if (prevTicks == 0) {
            prevTicks = currentTicks;
            prevTime = currentTime;
        }
        double tickDiff = currentTicks - prevTicks;
        double timeDiff = currentTime - prevTime;
        if (timeDiff > MINIMUM_TIME_DIFFERENCE) {
            rpm = (tickDiff / timeDiff) * (60000000000.0 / 28.0);//60,000,000,000 is nanoseconds per minute; 28 is ticks per rotation
            prevTicks = currentTicks;
            prevTime = currentTime;
            if (debug) {
                mode.telemetry.addData("RPM", rpm);
                mode.telemetry.update();
                Log.d("RPM", String.valueOf(rpm));
            }
            adjustRPM();
        }
    }

    private void adjustRPM() {
        double rpmDif = targetRPM - rpm;
        double powerDif = rpmDif / (MAX_RPM * ADJUSTMENT_RATE);
        double newPower = motor.getPower() + powerDif;

        if (targetRPM == 0) motor.setPower(0);

        motor.setPower(newPower);
    }
}
