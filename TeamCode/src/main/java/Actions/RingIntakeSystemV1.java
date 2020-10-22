package Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RingIntakeSystemV1 {

    private DcMotor wheelMotor;
    // private DcMotor armTurner;

    private static final double WHEEL_POWER = 1;
    private static final double SHOOTER_DIRECTION = 45;

    public RingIntakeSystemV1(HardwareMap hardwareMap) {
        wheelMotor = hardwareMap.dcMotor.get("wheelMotor");
        // armTurner = hardwareMap.dcMotor.get("armTurner");
    }

    public void toggleIntakePower() {
        wheelMotor.setPower(WHEEL_POWER);
    }

    public void toggleIntakeDirection() {
        // TODO
    }

}
