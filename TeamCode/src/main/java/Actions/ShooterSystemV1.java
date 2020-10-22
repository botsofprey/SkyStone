package Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSystemV1 {

    private DcMotor wheelMotor;
    // private DcMotor armTurner;

    private static final double WHEEL_POWER = 1;
    private static final double SHOOTER_DIRECTION = 45;

    public ShooterSystemV1(HardwareMap hardwareMap) {
        wheelMotor = hardwareMap.dcMotor.get("wheelMotor");
    }

    public void beginShooting() { wheelMotor.setPower(WHEEL_POWER); }

    public void stopShooting() { wheelMotor.setPower(0); }

    public void adjustShootingAngle() {}

    public void targetTopGoal() {}

    public void raiseHopper() {}
    public void lowerHopper() {}

//    public void raiseArm() {}
//    public void lowerArm() {}

}
