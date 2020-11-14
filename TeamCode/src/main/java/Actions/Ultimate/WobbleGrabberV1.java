package Actions.Ultimate;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;

import MotorControllers.MotorController;

/**
 * Author: Ethan Fisher
 * Date: 10/21/2020
 *
 * Used for grabbing and releasing the wobble goal
 */
public class WobbleGrabberV1 {

    // TODO test grabbing

    private Servo claw;
    public MotorController arm;
    private RevColorSensorV3 redSensor;

    private static final double ARM_POWER_UP = -0.8;
    private static final double ARM_POWER_DOWN = 0.1;

    private static final int ARM_DOWN_TICKS = 240;
    private static final int ARM_UP_TICKS = 0;

    private static final double CLAW_GRAB_ANGLE = 0.0;
    private static final double CLAW_RELEASE_ANGLE = 0.5;

    public static final int PIXELS_FOR_WOBBLE_GRAB = 0;

    private boolean goingUp;
    private boolean goingDown;

    public WobbleGrabberV1(HardwareMap hardwareMap) throws Exception {
        claw = hardwareMap.servo.get("wobbleGrabberClaw");

        // don't worry about this line. Just test what's currently in the file
        arm = new MotorController("wobbleGrabberArm", "ActionConfig/WobbleArmConfig.json", hardwareMap);

        redSensor = hardwareMap.get(RevColorSensorV3.class, "redSensor");

        goingUp = false;
        goingDown = false;
    }

    // TODO it's okay if after lifting, the motor begins to fall. Just try and get the servo to grab it and the motor to move up
    public void grabWobbleGoal() {
//        arm.setTargetPosition(ARM_DOWN_ANGLE);
//        arm.setPower(ARM_POWER);
//        while (arm.isBusy());
//        arm.setPower(0);
//
//        claw.setPosition(CLAW_GRAB_ANGLE);
//
//        arm.setTargetPosition(ARM_UP_ANGLE);
//        arm.setPower(-ARM_POWER);
//        while (arm.isBusy());
//        arm.setPower(0);

        lowerArm();
        claw.setPosition(CLAW_GRAB_ANGLE);
        raiseArm();
    }

    public void releaseWobbleGoal() {

//        arm.setTargetPosition(ARM_DOWN_ANGLE);
//        arm.setPower(ARM_POWER);
//        while (arm.isBusy());
//        arm.setPower(0);
//
//        claw.setPosition(CLAW_RELEASE_ANGLE);
//
//        arm.setTargetPosition(ARM_UP_ANGLE);
//        arm.setPower(-ARM_POWER);
//        while (arm.isBusy());

        lowerArm();
        claw.setPosition(CLAW_RELEASE_ANGLE);
        raiseArm();
    }

    public void raiseArm() {
        goingDown = false;
        goingUp = true;

        arm.setPositionTicks(ARM_UP_TICKS);
        arm.setMotorPower(ARM_POWER_UP);
    }

    public void lowerArm() {
        goingDown = true;
        goingUp = false;

        arm.setPositionTicks(ARM_DOWN_TICKS);
        arm.setMotorPower(ARM_POWER_DOWN);
    }

    public void holdPosition() { arm.holdPosition(); }

    public boolean isGoingUp() { return goingUp; }
    public boolean isGoingDown() { return goingDown; }

    public int getRedPixels() { return redSensor.red(); }

}