package Actions.Ultimate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Author: Ethan Fisher
 * Date: 10/21/2020
 *
 * Used for shooting rings
 */
public class ShooterSystemV1 {

    private DcMotor wheelMotor;
    private Servo hopperTurner;
    private Servo pinballServo;
    // private DcMotor armTurner;

    private static final double WHEEL_POWER = -1;

    // TODO put a variable here for the pinball's angle when it is shot

    private static final double HOPPER_UP_ANGLE = 30;
    private static final double HOPPER_DOWN_ANGLE = 0;

    public ShooterSystemV1(HardwareMap hardwareMap) {
        wheelMotor = hardwareMap.dcMotor.get("wheelMotor");
        hopperTurner = hardwareMap.servo.get("hopperTurner");
        pinballServo = hardwareMap.servo.get("pinballServer");
    }

    public void shoot() {
        // TODO rotate pinball servo
    }

    // TODO
    public void adjustShootingAngle() {}

    // TODO
    public void targetTopGoal() {}

    public void raiseHopper() {
        hopperTurner.setPosition(HOPPER_UP_ANGLE);

        // also, start the wheel motor
        wheelMotor.setPower(WHEEL_POWER);
    }
    public void lowerHopper() {
        hopperTurner.setPosition(HOPPER_DOWN_ANGLE);

        // also, stop the wheel motor
        wheelMotor.setPower(0);
    }

    public void raiseArm() {}
    public void lowerArm() {}

}
