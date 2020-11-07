package Actions.Ultimate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RingIntakeSystemV1 {

//    private DcMotor intakeMotor;
    private boolean intakeOn;
    private boolean intakeReversed;

    private static final int MOTOR_OFF_POWER = 0;
    private static final int MOTOR_ON_POWER = 1;

    public RingIntakeSystemV1(HardwareMap hardwareMap) {
//        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        intakeOn = false;
        intakeReversed = false;
    }

    public void toggleIntakePower() {
        // Turn intake motor on or off
        intakeOn = !intakeOn;
//        intakeMotor.setPower(intakeOn ? MOTOR_ON_POWER : MOTOR_OFF_POWER);
    }

    public void toggleIntakeDirection() {
        // Reverse direction of intake motor
        intakeReversed = !intakeReversed;
//        intakeMotor.setPower(intakeMotor.getPower() * (intakeReversed ? -1 : 1));
    }


}
