package Actions.Ultimate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import Autonomous.ConfigVariables;
import Autonomous.Location;
import Autonomous.Vector3;

/**
 * Author: Ethan Fisher
 * Date: 10/21/2020
 *
 * Used for shooting rings
 */
public class ShooterSystemV1 {

    // TODO test
    private Servo aimServo;
    private double aimPosition;
    private static final double TOP_GOAL_POSITION = 1;
    private static final double POWER_SHOT_POSITION = 0.8;
    private static final double LOWERED_POSITION = 0.5;

    // good
    private DcMotor wheelMotor;
    private boolean wheelSpinning;
    private static final double SHOOTER_ON_POWER = -1;
    private static final double SHOOTER_OFF_POWER = 0;

    // TODO test
    private Servo elevatorServo;
    private double elevatorAngle;
    private static final double ELEVATOR_UP_ANGLE = 1;
    private static final double ELEVATOR_DOWN_ANGLE = 0;

    // good
    private Servo pinballServo;
    private double pinballAngle;
    private static final double PINBALL_TURNED = 1;
    private static final double PINBALL_REST = 0;

    public ShooterSystemV1(HardwareMap hardwareMap) {
        aimServo = hardwareMap.servo.get("aimServo");
        wheelMotor = hardwareMap.dcMotor.get("wheelMotor");
        elevatorServo = hardwareMap.servo.get("elevatorServo");
        pinballServo = hardwareMap.servo.get("pinballServo");

        wheelSpinning = false;
        aimPosition = LOWERED_POSITION;
        pinballAngle = PINBALL_REST;
        elevatorAngle = ELEVATOR_DOWN_ANGLE;
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
        // TODO switch to shoot with adjusted angle
        if (pinballAngle == PINBALL_TURNED) pinballAngle = PINBALL_REST;
        else pinballAngle = PINBALL_TURNED;

        pinballServo.setPosition(pinballAngle);
    }

    public void adjustShootingAngle() {
        if (aimPosition == TOP_GOAL_POSITION) aimPosition = POWER_SHOT_POSITION;
        else if (aimPosition == POWER_SHOT_POSITION) aimPosition = LOWERED_POSITION;
        else aimPosition = TOP_GOAL_POSITION;

        aimServo.setPosition(aimPosition);
    }

    public void adjustHopperAngle() {
        if (elevatorAngle == ELEVATOR_UP_ANGLE) elevatorAngle = ELEVATOR_DOWN_ANGLE;
        else elevatorAngle = ELEVATOR_UP_ANGLE;

        elevatorServo.setPosition(elevatorAngle);
    }

    // TODO
    public void shootWithAdjustedAngle(Location robotLocation) {
        // targets: top goal, powershots

        double shooterHeightCM = 5;
        Vector3 robotVector = new Vector3(robotLocation.getX(), robotLocation.getY(), shooterHeightCM);

        Vector3 targetVector;
        if (aimPosition == POWER_SHOT_POSITION) {
            targetVector = new Vector3(ConfigVariables.POWER_SHOT_MIDDLE.getX(),
                    ConfigVariables.POWER_SHOT_MIDDLE.getY(),
                    ConfigVariables.POWER_SHOT_HEIGHT_CM);
        } else {
            targetVector = new Vector3(ConfigVariables.TOP_GOAL.getX(),
                    ConfigVariables.TOP_GOAL.getY(),
                    ConfigVariables.TOP_GOAL_HEIGHT_CM);
        }

        Vector3 differenceVector = robotVector.distanceFromVector(targetVector);
        double distanceFromTargetCM = differenceVector.length();

        double motorPower = calculateMotorPower(distanceFromTargetCM);
        double servoAngle = calculateShootingAngle(motorPower, distanceFromTargetCM);

        wheelMotor.setPower(motorPower);
        aimServo.setPosition(servoAngle);
    }

    // TODO
    public double calculateMotorPower(double distanceCM) {
        // constrain it to like 0.6 to 1 or something
        return 0;
    }

    // TODO
    public double calculateShootingAngle(double motorPower, double distance) {
        // probably gonna be something like 30 - 40 degrees
        return 0;
    }
}
