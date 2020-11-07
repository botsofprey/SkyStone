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

    private static final int ARM_GRAB_ANGLE = 45;
    private static final int ARM_RELEASE_ANGLE = 0;

    private static final double CLAW_GRAB_ANGLE = 0.1;
    private static final double CLAW_RELEASE_ANGLE = 0;

    public WobbleGrabberV1(HardwareMap hardwareMap) {
        claw = hardwareMap.servo.get("wobbleGrabberClaw");
        arm = hardwareMap.dcMotor.get("wobbleGrabberArm");
    }

    public void grabWobbleGoal() {
        // set position of arm to ARM GRAB ANGLE
        // grab with the claw
        arm.setTargetPosition(ARM_GRAB_ANGLE);
        claw.setPosition(CLAW_GRAB_ANGLE);
    }

    public void releaseWobbleGoal() {
        // release with claw
        // move arm to ARM RELEASE ANGLE
        arm.setTargetPosition(ARM_RELEASE_ANGLE);
        claw.setPosition(CLAW_RELEASE_ANGLE);
    }

}

