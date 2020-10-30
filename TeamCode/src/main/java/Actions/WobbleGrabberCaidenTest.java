package Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleGrabberCaidenTest {

    private Servo claw;
    private DcMotor arm;

    private static final int ARM_HOLD_ANGLE = 45;
    private static final int ARM_RELEASE_ANGLE = 0;
    private static final int CLAW_OPEN_ANGLE = 75; //105
    private static final int CLAW_CLOSE_ANGLE = 105; //75

    private static final int UP_POSITION = ARM_HOLD_ANGLE / 180;
    private static final int DOWN_POSITION = ARM_RELEASE_ANGLE / 180;
    private static final int OPEN_POSITION = CLAW_OPEN_ANGLE / 180;
    private static final int CLOSE_POSITION = CLAW_CLOSE_ANGLE / 180;

    public WobbleGrabberCaidenTest(HardwareMap hardwareMap) {
        claw = hardwareMap.servo.get("wobbleGrabberClaw");
        arm = hardwareMap.dcMotor.get("wobbleGrabberArm");

    }

    public void grabWobbleGoal() {
        // move arm to down position
        // move claw to closed position
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition(DOWN_POSITION);
        arm.setPower(.3);
        claw.setPosition(OPEN_POSITION);
        claw.setPosition(CLOSE_POSITION);
    }

    public void releaseWobbleGoal() {
        // move arm to down position
        // move claw to open position
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition(DOWN_POSITION);
        arm.setPower(.3);
        claw.setPosition(OPEN_POSITION);
    }
}
