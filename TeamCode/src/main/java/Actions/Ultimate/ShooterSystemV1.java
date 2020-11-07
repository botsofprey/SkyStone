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

    private DcMotor aimMotor;
    private int armPosition;
    private static final int TOP_GOAL_POSITION = 30;
    private static final int POWER_SHOT_POSITION = 20;
    private static final int LOWERED_POSITION = 0;

    private DcMotor wheelMotor;
    private boolean wheelSpinning;
    private static final double SHOOTER_ON_POWER = -1;
    private static final double SHOOTER_OFF_POWER = 0;

    private Servo hopperTurner;
    private double hopperAngle;
    private static final double HOPPER_UP_ANGLE = 1;
    private static final double HOPPER_DOWN_ANGLE = 0;

    private Servo pinballServo;
    private double pinballAngle;
    private static final double PINBALL_TURNED = 0.25;
    private static final double PINBALL_REST = 0;

    public ShooterSystemV1(HardwareMap hardwareMap) {
        wheelMotor = hardwareMap.dcMotor.get("wheelMotor");
        aimMotor = hardwareMap.dcMotor.get("aimMotor");
        hopperTurner = hardwareMap.servo.get("hopperTurner");
        pinballServo = hardwareMap.servo.get("pinballServo");

        wheelSpinning = false;
        armPosition = LOWERED_POSITION;
        pinballAngle = PINBALL_REST;
        hopperAngle = HOPPER_DOWN_ANGLE;
    }

    public void toggleWheelPower() {
        wheelSpinning = !wheelSpinning;
        wheelMotor.setPower(wheelSpinning ? SHOOTER_ON_POWER : SHOOTER_OFF_POWER);
    }

    public void turnOnShooterWheel() {
        wheelSpinning = true;
        wheelMotor.setPower(SHOOTER_ON_POWER);
    }

    public void turnOffShooterWheel() {
        wheelSpinning = false;
        wheelMotor.setPower(SHOOTER_OFF_POWER);
    }

    // moves the pinball servo
    public void shoot() {
        if (pinballAngle == PINBALL_TURNED) pinballAngle = PINBALL_REST;
        else pinballAngle = PINBALL_TURNED;

        pinballServo.setPosition(pinballAngle);
    }

    public void adjustShootingAngle() {
        if (armPosition == TOP_GOAL_POSITION) armPosition = POWER_SHOT_POSITION;
        else if (armPosition == POWER_SHOT_POSITION) armPosition = LOWERED_POSITION;
        else armPosition = TOP_GOAL_POSITION;

        aimMotor.setPower(0.3);
    }

    public void adjustHopperAngle() {
        if (hopperAngle == HOPPER_UP_ANGLE) hopperAngle = HOPPER_DOWN_ANGLE;
        else hopperAngle = HOPPER_UP_ANGLE;

        hopperTurner.setPosition(hopperAngle);
    }

    public void shootWithAdjustedAngle(double distanceFromTarget, double targetHeight) {
        // targets: top goal, powershots
        double heightToShoot = targetHeight - 0; // height of shooter
    }

    public double calculateMotorPower(double distance) {
        return 0;
    }

    public double calculateShootingAngle(double motorPower, double distance) {
        return 0;
    }

}
