package Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleGrabberCaidenTest {

    private Servo claw;
    private DcMotor arm;

    private static final int ARM_GRAB_ANGLE = 45;
    private static final int ARM_RELEASE_ANGLE = 0;
    private static final int CLAW_OPEN_ANGLE = 75; //105
    private static final int CLAW_CLOSE_ANGLE = 105; //75

    public WobbleGrabberCaidenTest(HardwareMap hardwareMap) {
        claw = hardwareMap.servo.get("wobbleGrabberClaw");
        arm = hardwareMap.dcMotor.get("wobbleGrabberArm");

    }

    public void grabWobbleGoal() {
        // set position of arm to ARM GRAB ANGLE
        // grab with the claw
        arm.setMode(DcMotor.RunMode.RUN_WITH_ENCODER);
        arm.setTargetPosition(ARM_GRAB_ANGLE);
        arm.setPower(.3);
        claw.setAngle(CLAW_OPEN_ANGLE);
        while (opModeIsActive() && arm.isBusy()) {idle();}
        claw.setAngle(CLAW_CLOSE_ANGLE);
    }

    public void releaseWobbleGoal() {
        // release with claw
        // move arm to ARM RELEASE ANGLE
        arm.setMode(DcMotor.RunMode.RUN_WITH_ENCODER);
        arm.setTargetPosition(ARM_RELEASE_ANGLE);
        arm.setPower(.3);
        while (opModeIsActive() && leftMotor.isBusy()) {idle();}
        claw.setAngle(CLAW_OPEN_ANGLE);
    }

}

