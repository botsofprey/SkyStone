package Actions.Ultimate;

import android.sax.StartElementListener;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import SensorHandlers.MagneticLimitSwitch;
import UserControlled.GamepadController;

/**
 * Author: Ethan Fisher
 * Date: 10/21/2020
 *
 * Used for shooting rings
 */
public class ShooterSystemV1 {

    // good
    private Servo aimServo;
    public static final double HIGHEST_POSITION = 0;
    public static final double LOWERED_POSITION = 1;

    // good
    private DcMotor wheelMotor;
    private boolean wheelSpinning;
    private static double rpm;
    private static long ticks;
    private static long nanoseconds;
    private static int iterator;
    private static double[] rpmMeasurements;
    private static final double SHOOTER_ON_POWER = 1;
    private static final double SHOOTER_OFF_POWER = 0;

    // good
    private CRServo elevatorServo;
    public static final int TOP = 0;
    public static final int BOTTOM = 1;
//    public static final int MOVING = 2;

    public int elevatorPosition;
    private MagneticLimitSwitch elevatorTopSwitch;
    private MagneticLimitSwitch elevatorBottomSwitch;

    // good
    public Servo pinballServo;
    private double pinballAngle;
    public static final double PINBALL_TURNED = 1;
    public static final double PINBALL_REST = 0;

    public ShooterSystemV1(HardwareMap hardwareMap) {
        aimServo = hardwareMap.servo.get("aimServo");
        wheelMotor = hardwareMap.dcMotor.get("wheelMotor");
        elevatorServo = hardwareMap.crservo.get("elevatorServo");
        elevatorTopSwitch = new MagneticLimitSwitch(hardwareMap.digitalChannel.get("elevatorTopSwitch"));
        elevatorBottomSwitch = new MagneticLimitSwitch(hardwareMap.digitalChannel.get("elevatorBottomSwitch"));

        pinballServo = hardwareMap.servo.get("pinballServo");

        wheelSpinning = false;
        elevatorPosition = BOTTOM;
        pinballAngle = PINBALL_REST;
        rpm = 0;
        ticks = 0;
        nanoseconds = System.nanoTime();
        iterator = 0;
        rpmMeasurements = new double[1024];
        for(int i = 0; i < 1024; i++)
            rpmMeasurements[i] = 0;

        new Thread(new Runnable() {
            @Override
            public void run() {
                long currentTicks = wheelMotor.getCurrentPosition();
                long currentNanoseconds = System.nanoTime();
                if(currentTicks != ticks) {//only calculate if ticks have changed to avoid outputting 0
                    double timeDiff = currentNanoseconds - nanoseconds;
                    double nanosecondsPerRotation = timeDiff / (currentTicks - ticks) * 28;//28 encoder ticks per rotation
                    double minutesPerRotation = nanosecondsPerRotation / 60000000000.0;
                    rpmMeasurements[iterator] = 1.0 / minutesPerRotation;
                    iterator++;
                    iterator %= 1024;
                    ticks = currentTicks;
                    nanoseconds = currentNanoseconds;
                    double temp = 0;
                    for(int i = 0; i < 1024; i++)
                        temp += rpmMeasurements[i];
                    rpm = temp / 1024.0;
                }
            }
        }).start();
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

    public void updateShooterRPM() {
        long currentTicks = wheelMotor.getCurrentPosition();
        long currentNanoseconds = System.nanoTime();
        if(currentTicks != ticks) {//only calculate if ticks have changed to avoid outputting 0
            long timeDiff = currentNanoseconds - nanoseconds;
            double nanosecondsPerRotation = timeDiff * 28;//28 encoder ticks per rotation
            double minutesPerRotation = nanosecondsPerRotation / 60000000000.0;
            rpm = 1.0 / minutesPerRotation;
            ticks = currentTicks;
            nanoseconds = currentNanoseconds;
        }
    }

    // moves the pinball servo
    public void shoot() {
        if (pinballAngle == PINBALL_TURNED)
            pinballAngle = PINBALL_REST;
        else
            pinballAngle = PINBALL_TURNED;

        pinballServo.setPosition(pinballAngle);
    }

    public void raiseShooter(double angle) {
        aimServo.setPosition(aimServo.getPosition() - angle);
    }

    public void lowerShooter(double angle) {
        aimServo.setPosition(aimServo.getPosition() + angle);
    }

    public void setShooter(double angle) { aimServo.setPosition(angle); }

    public void raiseElevator() {
        if (elevatorPosition != TOP)
            elevatorServo.setPower(-1);
    }

    public void lowerElevator() {
        if (elevatorPosition != BOTTOM ) {
            elevatorServo.setPower(1);
        }
    }

    public void stopElevator() {
        elevatorServo.setPower(0);
    }

    public void update(LinearOpMode mode) {
        if (elevatorTopSwitch.isActivated() && elevatorPosition != TOP) {
            elevatorPosition = TOP;
            elevatorServo.setPower(0);
        } else if (elevatorBottomSwitch.isActivated() && elevatorPosition != BOTTOM) {
            elevatorPosition = BOTTOM;
            elevatorServo.setPower(0);
        }

        mode.telemetry.addData("RPM", rpm);
        mode.telemetry.addData("Bottom Activated", elevatorBottomSwitch.isActivated());
        mode.telemetry.addData("Top Activated", elevatorTopSwitch.isActivated());
    }

    // TODO
//    public void shootWithAdjustedAngle(Location robotLocation) {
//        // targets: top goal, powershots
//
//        double shooterHeightCM = 5;
//        Vector3 robotVector = new Vector3(robotLocation.getX(), robotLocation.getY(), shooterHeightCM);
//
//        Vector3 targetVector;
//        if (aimPosition == POWER_SHOT_POSITION) {
//            targetVector = new Vector3(ConfigVariables.POWER_SHOT_MIDDLE.getX(),
//                    ConfigVariables.POWER_SHOT_MIDDLE.getY(),
//                    ConfigVariables.POWER_SHOT_HEIGHT_CM);
//        } else {
//            targetVector = new Vector3(ConfigVariables.TOP_GOAL.getX(),
//                    ConfigVariables.TOP_GOAL.getY(),
//                    ConfigVariables.TOP_GOAL_HEIGHT_CM);
//        }
//
//        Vector3 differenceVector = robotVector.distanceFromVector(targetVector);
//        double distanceFromTargetCM = differenceVector.length();
//
//        double motorPower = calculateMotorPower(distanceFromTargetCM);
//        double servoAngle = calculateShootingAngle(motorPower, distanceFromTargetCM);
//
//        wheelMotor.setPower(motorPower);
//        aimServo.setPosition(servoAngle);
//    }
//
//    // TODO
//    public double calculateMotorPower(double distanceCM) {
//        // constrain it to like 0.6 to 1 or something
//        return 0;
//    }
//
//    // TODO
//    public double calculateShootingAngle(double motorPower, double distance) {
//        // probably gonna be something like 30 - 40 degrees
//        return 0;
//    }
}
