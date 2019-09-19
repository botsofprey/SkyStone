package Actions;

import android.drm.DrmStore;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import Actions.HardwareWrappers.ServoHandler;

public class RakeSystem implements ActionHandler {
    private HardwareMap hardwareMap;
    private ServoHandler rake, rakeArm;

    public RakeSystem(HardwareMap hw) {
        hardwareMap = hw;
        rake = new ServoHandler("rake", hardwareMap);
        rakeArm = new ServoHandler("rakearm", hardwareMap);
        rake.setDirection(Servo.Direction.FORWARD);
        rakeArm.setDirection(Servo.Direction.FORWARD);
        rake.setServoRanges(0, 180);
        rakeArm.setServoRanges(0, 180);
        rakeArm.setDegree(0);
    }

    public void closeArm() {rakeArm.setDegree(180);}
    public void openArm() {rakeArm.setDegree(0);}
    public void setArmDegree(double deg) {rakeArm.setDegree(deg);}

    public void extendRake() {rake.setPosition(1);}
    public void retractRake() {rake.setPosition(0);}
    public void pauseRake() {rake.setPosition(0.5);}

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
        pauseRake();
        openArm();
    }
}
