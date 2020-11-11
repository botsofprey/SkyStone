package Actions.Ultimate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Author: Ethan Fisher
 * Date: 10/21/2020
 *
 * Used for grabbing and releasing the wobble goal
 */
public class WobbleGrabberV1 {

    private Servo claw;
    private DcMotor arm;

    private static final double ARM_POWER = 0.2;

    private static final int ARM_GRAB_ANGLE = 45;
    private static final int ARM_RELEASE_ANGLE = 0;

    private static final double CLAW_GRAB_ANGLE = 1.0;
    private static final double CLAW_RELEASE_ANGLE = 0.5;

    public WobbleGrabberV1(HardwareMap hardwareMap) {
        claw = hardwareMap.servo.get("wobbleGrabberClaw");
        arm = hardwareMap.dcMotor.get("wobbleGrabberArm");
    }

    public void grabWobbleGoal() {
        arm.setTargetPosition(ARM_GRAB_ANGLE);
        arm.setPower(ARM_POWER);
        claw.setPosition(CLAW_GRAB_ANGLE);
    }

    public void releaseWobbleGoal() {
        arm.setPower(-ARM_POWER);
        arm.setTargetPosition(ARM_RELEASE_ANGLE);
        claw.setPosition(CLAW_RELEASE_ANGLE);
    }

}