package Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.io.IOException;

import Actions.HardwareWrappers.ServoHandler;
import Actions.HardwareWrappers.SpoolMotor;
import MotorControllers.MotorController;

public class StoneStackingSystemV2 implements ActionHandler{

    HardwareMap hardwareMap;
    SpoolMotor lift;
    MotorController leftRake, rightRake;
    ServoHandler leftBlenderFoot, rightBlenderFoot, centralGripper, levatronArm, foundationGrabber;
    TouchSensor left, right;
    public static final double LEFT_FOOT_STORED = 0, LEFT_FOOT_RELEASE = 90, LEFT_FOOT_GRAB = 180;
    public static final double RIGHT_FOOT_STORED = 0, RIGHT_FOOT_RELEASE = 90, RIGHT_FOOT_GRAB = 180;
    public static final double CENTRAL_ARM_GRAB = 0, CENTRAL_ARM_RELEASE = 120, CENTRAL_ARM_DEPLOY_BUMPER = 180, LEVATRON_SET = 0, LEVATRON_RELEASE = 180;
    public static final double FOUNDATION_GRAB = 0, FOUNDATION_RELEASE = 180;
    public static final double STONE_HEIGHT_1 = 1.74, STONE_HEIGHT_2 = 3.9, STONE_HEIGHT_3 = 5.9, STONE_HEIGHT_4 = 7.75;
    public StoneStackingSystemV2(HardwareMap hw) {
        hardwareMap = hw;
        try {
            lift = new SpoolMotor(new MotorController("lift", "ActionConfig/SSSLift.json", hardwareMap), 50, 50, 35, hardwareMap);
            leftRake = new MotorController("leftRake", "ActionConfig/RakeMotor.json", hardwareMap);
            rightRake = new MotorController("rightRake", "ActionConfig/RakeMotor.json", hardwareMap);
            left = hardwareMap.touchSensor.get("leftTouchSensor");
            right = hardwareMap.touchSensor.get("rightTouchSensor");

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
        leftBlenderFoot = new ServoHandler("leftBlenderFoot", hardwareMap);
        rightBlenderFoot = new ServoHandler("rightBlenderFoot", hardwareMap);
        centralGripper = new ServoHandler("centralGripper", hardwareMap);
        levatronArm = new ServoHandler("levatronArm", hardwareMap);
        foundationGrabber = new ServoHandler("foundationGrabber", hardwareMap);
        leftBlenderFoot.setServoRanges(1, 179);
        rightBlenderFoot.setServoRanges(1, 179);
        centralGripper.setServoRanges(1, 179);
        levatronArm.setServoRanges(1, 179);
        foundationGrabber.setServoRanges(1, 179);
        leftBlenderFoot.setDegree(LEFT_FOOT_STORED);
        rightBlenderFoot.setDirection(Servo.Direction.REVERSE);
        rightBlenderFoot.setDegree(RIGHT_FOOT_STORED);
        centralGripper.setDegree(CENTRAL_ARM_RELEASE);
        levatronArm.setDegree(LEVATRON_SET);
        foundationGrabber.setDegree(FOUNDATION_RELEASE);
    }

    public void grabFoundation(){
        foundationGrabber.setDegree(FOUNDATION_GRAB);
    }

    public void releaseFoundation(){
        foundationGrabber.setDegree(FOUNDATION_RELEASE);
    }

    public void grabStoneWithBlenderFeet() {
        leftBlenderFoot.setDegree(LEFT_FOOT_GRAB);
        rightBlenderFoot.setDegree(RIGHT_FOOT_GRAB);
    }
    public void releaseStoneWithBlenderFeet() {
        leftBlenderFoot.setDegree(LEFT_FOOT_RELEASE);
        rightBlenderFoot.setDegree(RIGHT_FOOT_RELEASE);
    }
    public void setBlenderFeetDegrees(double leftDeg, double rightDeg) {
        leftBlenderFoot.setDegree(leftDeg);
        rightBlenderFoot.setDegree(rightDeg);
    }

    public void dropCapStone() {
        levatronArm.setDegree(LEVATRON_RELEASE);
    }
    public void resetCapStone() {
        levatronArm.setDegree(LEVATRON_SET);
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
    public void lowerStones() { lift.retract(); }
    public void pauseStoneLift() {
        lift.holdPosition();
    }
    public void liftToPosition(int pos) {
        switch (pos) {
            case 1:
                setLiftPosition(STONE_HEIGHT_1);
                break;
            case 2:
                setLiftPosition(STONE_HEIGHT_2);
                break;
            case 3:
                setLiftPosition(STONE_HEIGHT_3);
                break;
            case 4:
                setLiftPosition(STONE_HEIGHT_4);
                break;
            default:
                break;
        }
    }
    public void setLiftPosition(double posInInches) {
        lift.setPostitionInches(posInInches);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
    }
    public double getLiftPositionInches() { return lift.getPositionInches(); }
    public void resetLiftEncoder() {
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void extendLeftArm() { leftRake.setMotorPower(1); }
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
    public void pauseRightArm() { rightRake.brake(); }

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
