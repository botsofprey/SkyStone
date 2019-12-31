package Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.IOException;

import MotorControllers.MotorController;

public class StoneIntakeSystemV1 implements ActionHandler {
    HardwareMap hardwareMap;
    MotorController leftIntake, rightIntake;

    public StoneIntakeSystemV1(HardwareMap hw) {
        hardwareMap = hw;
        try {
            leftIntake = new MotorController("leftIntake", "ActionConfig/StoneIntake.json", hardwareMap);
            rightIntake = new MotorController("rightIntake", "ActionConfig/StoneIntake.json", hardwareMap);
            leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
            rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
            leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void intake() {
        leftIntake.setMotorPower(1);
        rightIntake.setMotorPower(1);
    }
    public void expel() {
        leftIntake.setMotorPower(-1);
        rightIntake.setMotorPower(-1);
    }
    public void pauseIntake() {
        leftIntake.brake();
        rightIntake.brake();
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
        leftIntake.killMotorController();
        rightIntake.killMotorController();
    }
}
