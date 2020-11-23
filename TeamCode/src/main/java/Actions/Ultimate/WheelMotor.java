package Actions.Ultimate;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class WheelMotor {

    public static final boolean debug = true;

    public DcMotor motor;
    private boolean wheelSpinning;
    private double rpm;
    private double targetRPM;
    private long prevTicks;
    private long prevTime;
    private static final int maxRPM = 6000;
    private static final double MINIMUM_TIME_DIFFERENCE = 100000000;
    private static final double adjustmentRate = 2;

    public WheelMotor(String name, HardwareMap hardwareMap) {
        motor = hardwareMap.dcMotor.get(name);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetRPM = 0;
    }

    public void setRPM(int RPM) {
        motor.setPower((double) RPM / (double) maxRPM);
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
            }
            adjustRPM(mode);
        }
    }

    private void adjustRPM(OpMode mode) {
        double rpmDif = targetRPM - rpm;
        double powerDif = rpmDif / (maxRPM * adjustmentRate);
        double newPower = motor.getPower() + powerDif;
        motor.setPower(newPower);
    }
}
