package Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterSystemV1 {

    private DcMotor wheelMotor;
    private Servo hopperTurner;
    // private DcMotor armTurner;

    private static final double WHEEL_POWER = -1;
    private static final double SHOOTER_DIRECTION = 45;

    private static final double HOPPER_UP = 30;
    private static final double HOPPER_DOWN = 0;

    public ShooterSystemV1(HardwareMap hardwareMap) {
        wheelMotor = hardwareMap.dcMotor.get("wheelMotor");
        hopperTurner = hardwareMap.servo.get("hopperTurner");
    }

    public void beginShooting() { wheelMotor.setPower(WHEEL_POWER); }

    public void stopShooting() { wheelMotor.setPower(0); }

    public void adjustShootingAngle() {}

    public void targetTopGoal() {}

    // speed is the value on the gampad (-1 -> 1)
    public void raiseHopper() {
        hopperTurner.setPosition(HOPPER_UP);
    }
    public void lowerHopper() {
        hopperTurner.setPosition(HOPPER_DOWN);
    }

    public void raiseArm() {}
    public void lowerArm() {}

}
