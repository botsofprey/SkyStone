package Actions.Ultimate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RingIntakeSystemV1 {
    private DcMotor intakeMotor;
    private static final int MOTOR_OFF_POWER = 0;
    private static final int MOTOR_ON_POWER = 1;

    public RingIntakeSystemV1(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
    }

    public void toggleIntakePower() {
        // Turn intake motor on or off
        if(intakeMotor.getPower() == MOTOR_ON_POWER){
            intakeMotor.setPower(MOTOR_OFF_POWER);
        }
        else {
            intakeMotor.setPower(MOTOR_ON_POWER);
        }
    }

    public void toggleIntakeDirection() {
        // Reverse direction of intake motor
        intakeMotor.setPower(-(intakeMotor.getPower()));
    }


}
