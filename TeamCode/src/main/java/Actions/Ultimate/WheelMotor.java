package Actions.Ultimate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class WheelMotor {

    public DcMotor motor;

    private static final int MAX_RPM = 6000;

    public WheelMotor(String name, HardwareMap hardwareMap) {
        motor = hardwareMap.dcMotor.get(name);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setRPM(int rpm) {
        motor.setPower(rpm / (double) MAX_RPM);
    }
}
