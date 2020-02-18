package Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.IOException;

import Actions.HardwareWrappers.DoubledSpoolMotor;
import Actions.HardwareWrappers.ServoHandler;
import Actions.HardwareWrappers.SpoolMotor;
import MotorControllers.MotorController;

import static Actions.HardwareWrappers.DoubledSpoolMotor.MOTOR_1;
import static Actions.HardwareWrappers.DoubledSpoolMotor.MOTOR_2;

public class StoneStackingSystemV3 implements ActionHandler {

    HardwareMap hardwareMap;
    DoubledSpoolMotor lift;
    ServoHandler centralGripper, capstoneDeployer;
    public static final double CENTRAL_ARM_GRAB = 0, CENTRAL_ARM_RELEASE = 110, CENTRAL_ARM_IN_BOT = 180;
    public static final double STONE_HEIGHT_1 = 1.74, STONE_HEIGHT_2 = 3.9, STONE_HEIGHT_3 = 5.9, STONE_HEIGHT_4 = 7.75;
    public StoneStackingSystemV3(HardwareMap hw) {
        hardwareMap = hw;
        lift = new DoubledSpoolMotor(new String[] {"liftMotor1", "liftMotor2"}, "ActionConfig/SSSLift.json", 50, 50, hardwareMap);
        centralGripper = new ServoHandler("centralGripper", hardwareMap);
        capstoneDeployer = new ServoHandler("capStoneDeployer", hardwareMap);
        centralGripper.setServoRanges(0, 180);
        capstoneDeployer.setServoRanges(0, 180);
        centralGripper.setDegree(CENTRAL_ARM_IN_BOT);
        capstoneDeployer.setDegree(0);
//        winchServo.setDegree(0); // 30 degrees in one turn on this servo
//        measureTapeSpitter.
    }

//    public void setWinchServoDegree(double degree) {winchServo.setDegree(degree);}

    public void deployCapstone() { capstoneDeployer.setDegree(180); }
    public void releaseCapstone() { capstoneDeployer.setDegree(0); }
    public void setCapstoneDegree(double degree) {capstoneDeployer.setDegree(degree); }

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
//    public void setLiftPosition(double posInInches) {
//        lift.setPostitionInches(posInInches);
//        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lift.setPower(1);
//    }
//    public void liftToPosition(int pos) {
//        switch (pos) {
//            case 1:
//                setLiftPosition(STONE_HEIGHT_1);
//                break;
//            case 2:
//                setLiftPosition(STONE_HEIGHT_2);
//                break;
//            case 3:
//                setLiftPosition(STONE_HEIGHT_3);
//                break;
//            case 4:
//                setLiftPosition(STONE_HEIGHT_4);
//                break;
//            default:
//                break;
//        }
//    }
//    public double getLiftPositionInches() { return lift.getPositionInches(); }
//    public long getLiftPositionTicks() { return lift.getPosition(); }
    public void resetLiftEncoder() {
        lift.setPower(0);
        lift.setMode(MOTOR_1, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(MOTOR_2, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
