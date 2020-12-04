package Actions.Ultimate;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import MotorControllers.PIDController;

public class WheelMotor {

    public DcMotor motor;
    private LinearOpMode mode;
    private double rpm;
    private double targetRPM;
    private long prevTicks;
    private long prevTime;

    private static final double MAX_RPM = 6000;
    private static final double MINIMUM_TIME_DIFFERENCE = 100000000;// 1/10 of a second
    private static final long NANOS_PER_MINUTE = 60000000000L;
    private static final double TICKS_PER_ROTATION = 28;
    private static final double ADJUSTMENT_RATE = 2;

    private PIDController rpmController;

    public WheelMotor(String name, HardwareMap hardwareMap, final LinearOpMode mode) {
        motor = hardwareMap.dcMotor.get(name);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targetRPM = 0;

//        rpmController = new PIDController(0.1, 0.1, 0);
//        rpmController.setSp(0);

        this.mode = mode;
    }

    public void setRPM(int RPM) {
        targetRPM = RPM;
//        mode.telemetry.addData("Motor Power Init", "" + targetPower);
//        mode.telemetry.update();
        motor.setPower(targetRPM / MAX_RPM);

//        rpmController.setSp(RPM);
    }

    public void updateShooterRPM() {
        int currentTicks = motor.getCurrentPosition();
        long currentTime = System.nanoTime();
        if (prevTicks == 0) {
            prevTicks = currentTicks;
            prevTime = currentTime;
        }
        long tickDiff = currentTicks - prevTicks;
        long timeDiff = currentTime - prevTime;
        if (timeDiff > MINIMUM_TIME_DIFFERENCE) {
            rpm = (tickDiff / (double) timeDiff) * (NANOS_PER_MINUTE / TICKS_PER_ROTATION);
            prevTicks = currentTicks;
            prevTime = currentTime;
            mode.telemetry.addData("RPM", rpm);
            Log.d("RPM", "" + rpm);
            adjustRPM();
        }
    }

    private void adjustRPM() {
        double rpmDif = targetRPM - rpm;
        double powerDif = rpmDif / (MAX_RPM * ADJUSTMENT_RATE);
        double newPower = motor.getPower() + powerDif;

        if (targetRPM == 0)
            motor.setPower(0);
        else
            motor.setPower(newPower);

//        double curRPM = (dTicks / (double) dt) * (NANOS_PER_MINUTE / TICKS_PER_ROTATION);
//        double rpmCorrection = rpmController.calculatePID(curRPM);

//        motor.setPower((curRPM + rpmCorrection) / MAX_RPM);

//        mode.telemetry.addData("Motor Power", "" + targetPower);
//        mode.telemetry.update();
    }
}
