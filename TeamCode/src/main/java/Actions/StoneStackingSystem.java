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
    ServoHandler bottomGripper, topGripper;
    SpoolMotor stoneLifter;

    public final int BOTTOM_GRAB_DEGREE = 0, TOP_GRAB_DEGREE = 0, BOTTOM_RELEASE_DEGREE = 180, TOP_RELEASE_DEGREE = 180;

    public StoneStackingSystem(HardwareMap hw) {
        hardwareMap = hw;
        bottomGripper = new ServoHandler ("bottomGripper", hardwareMap);
        topGripper = new ServoHandler ("topGripper", hardwareMap);
        bottomGripper.setDirection(Servo.Direction.FORWARD);
        bottomGripper.setServoRanges(1,179);
        topGripper.setDirection(Servo.Direction.FORWARD);
        topGripper.setServoRanges(1,179);
        try {
            stoneLifter = new SpoolMotor (new MotorController("stoneLifter", "", hardwareMap), 25, 25, 100, hardwareMap);
            stoneLifter.setDirection(DcMotorSimple.Direction.FORWARD);
            stoneLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void liftStones () {
        stoneLifter.extendWithPower();
    }
    public void lowerStones () { stoneLifter.retractWithPower(); }

    public void pauseLift () { stoneLifter.pause(); }

    public void grabStone () {
        bottomGripper.setDegree(BOTTOM_GRAB_DEGREE);
        topGripper.setDegree(TOP_GRAB_DEGREE);
    }

    public void releaseStone () {
        bottomGripper.setDegree(BOTTOM_RELEASE_DEGREE);
        topGripper.setDegree(TOP_RELEASE_DEGREE);
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
