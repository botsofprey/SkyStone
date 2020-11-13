package Actions.Ultimate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import MotorControllers.MotorController;

/**
 * Author: Ethan Fisher
 * Date: 10/21/2020
 *
 * Used for grabbing and releasing the wobble goal
 */
public class WobbleGrabberV1 {

    private Servo claw;
    private MotorController arm;

    private static final double ARM_POWER = 0.2;

    private static final int ARM_DOWN_ANGLE = 45;
    private static final int ARM_UP_ANGLE = 0;

    private static final double CLAW_GRAB_ANGLE = 1.0;
    private static final double CLAW_RELEASE_ANGLE = 0.5;

    public WobbleGrabberV1(HardwareMap hardwareMap){
        claw = hardwareMap.servo.get("wobbleGrabberClaw");
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // don't worry about this line. Just test what's currently in the file
        arm = new MotorController("wobbleGrabberArm", "ActionConfig/DefaultMotorConfig.json", hardwareMap);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        arm.setPositionDegrees(ARM_UP_ANGLE);
        arm.setMotorPower(-ARM_POWER);
    }

    public void lowerArm() {
        arm.setPositionDegrees(ARM_DOWN_ANGLE);
        arm.setMotorPower(ARM_POWER);
    }

}