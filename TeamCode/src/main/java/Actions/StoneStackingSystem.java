package Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.IOException;

import Actions.HardwareWrappers.ServoHandler;
import Actions.HardwareWrappers.SpoolMotor;
import MotorControllers.MotorController;

public class StoneStackingSystem implements ActionHandler{
    HardwareMap hardwareMap;
    SpoolMotor lift;
    MotorController leftRake, rightRake;
    ServoHandler leftRakeArm, rightRakeArm, centralGripper;
    public static final double LEFT_ARM_STOP = 0.5, RIGHT_ARM_STOP = 0.5, LEFT_ARM_DEPLOY = 1, RIGHT_ARM_DEPLOY = 1, LEFT_ARM_RAISE = 0, RIGHT_ARM_RAISE = 0;
    public static final double CENTRAL_ARM_GRAB = 41, CENTRAL_ARM_RELEASE = 169;

    public StoneStackingSystem(HardwareMap hw) {
        hardwareMap = hw;
        try {
            lift = new SpoolMotor(new MotorController("lift", "ActionConfig/LiftMotor.json", hardwareMap), 50, 50, 35, hardwareMap);
            leftRake = new MotorController("leftRake", "ActionConfig/RakeMotor.json", hardwareMap);
            rightRake = new MotorController("rightRake", "ActionConfig/RakeMotor.json", hardwareMap);

            lift.setDirection(DcMotorSimple.Direction.REVERSE);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRake.setDirection(DcMotorSimple.Direction.REVERSE);
            leftRake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRake.setDirection(DcMotorSimple.Direction.REVERSE);
            rightRake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (IOException e) {
            e.printStackTrace();
        }
        leftRakeArm = new ServoHandler("leftRakeArm", hardwareMap);
        rightRakeArm = new ServoHandler("rightRakeArm", hardwareMap);
        centralGripper = new ServoHandler("centralGripper", hardwareMap);
        leftRakeArm.setServoRanges(1, 179);
        rightRakeArm.setServoRanges(1, 179);
        centralGripper.setServoRanges(1, 179);
        leftRakeArm.setPosition(LEFT_ARM_STOP);
        rightRakeArm.setPosition(RIGHT_ARM_STOP);
        centralGripper.setDegree(CENTRAL_ARM_RELEASE);
    }

    public void stopArms() {
        leftRakeArm.setPosition(LEFT_ARM_STOP);
        rightRakeArm.setPosition(RIGHT_ARM_STOP);
    }
    public void deployArms() {
        leftRakeArm.setPosition(LEFT_ARM_DEPLOY);
        rightRakeArm.setPosition(RIGHT_ARM_DEPLOY);
    }
    public void liftArms() {
        leftRakeArm.setPosition(LEFT_ARM_RAISE);
        rightRakeArm.setPosition(RIGHT_ARM_RAISE);
    }
    public void setLeftArmPosition(double pos) {
        leftRakeArm.setPosition(pos);
    }
    public void setRightArmPosition(double pos) {
        rightRakeArm.setPosition(pos);
    }

    public void setCentralGripperDegree(double deg) {
        centralGripper.setDegree(deg);
    }
    public void grabStoneCenter() {
        centralGripper.setDegree(CENTRAL_ARM_GRAB);
    }
    public void releaseStoneCenter() {
        centralGripper.setDegree(CENTRAL_ARM_RELEASE);
    }

    public void liftStones() {
        lift.extend();
    }
    public void lowerStones() {
        lift.retract();
    }
    public void pauseStoneLift() {
        lift.pause();
    }

    public void extendLeftArm() {
        leftRake.setMotorPower(1);
    }
    public void retractLeftArm() {
        leftRake.setMotorPower(-1);
    }
    public void pauseLeftArm() {
        leftRake.brake();
    }

    public void extendRightArm() {
        rightRake.setMotorPower(1);
    }
    public void retractRightArm() {
        rightRake.setMotorPower(-1);
    }
    public void pauseRightArm() {
        rightRake.brake();
    }

    @Override
    public boolean doAction(String action, long maxTimeAllowed) {
        return false;
    }

    @Override
    public boolean stopAction(String action) {
        return false;
    }

    @Override
    public boolean startDoingAction(String action) {
        return false;
    }

    @Override
    public void kill() {

    }
}
