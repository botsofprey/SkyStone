package Actions.Ultimate;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class WheelMotor {

    public DcMotor motor;
    private boolean wheelSpinning;
    private double rpm;
    private long prevTicks;
    private long prevTime;
    private int maxRPM;
    private boolean setMaxRPM;
    private static final double MINIMUM_TIME_DIFFERENCE = 100000000;

    public WheelMotor(String name, HardwareMap hardwareMap) {
        motor = hardwareMap.dcMotor.get(name);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setRPM(int RPM) {
        motor.setPower((double) RPM / (double) maxRPM);
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
            rpm = (tickDiff / timeDiff) * (60000000000.0 / 28.0);
            prevTicks = currentTicks;
            prevTime = currentTime;
            if(setMaxRPM)
                maxRPM = (int)rpm;
        }


        if (maxRPM == 0 && motor.getPower() == 1 && rpm == 0)
            updateShooterRPM(mode);
        else if (maxRPM == 0 && motor.getPower() == 1) {
            updateShooterRPM(mode);
            setMaxRPM = true;
        } else if (maxRPM != 0)
            updateShooterRPM(mode);
    }
}
