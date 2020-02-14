package Actions.HardwareWrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Actions.ActionHandler;
import MotorControllers.MotorController;

public class DoubledSpoolMotor implements ActionHandler {
    MotorController[] motors = new MotorController[2];

    HardwareMap hardwareMap;
    private double extendSpeedInPerSecond = 0;
    private double retractSpeedInPerSecond = 0;
    private double extendPower = 0.5;
    private long[] motorStartTicks;
    private double maxExtendLoc;
    public static final int MOTOR_1 = 0, MOTOR_2 = 1; //Grant wants 10000 and 20000 but... arrays :(
    public void DoubledSpoolMotor(double extendInPerSec, double retractInPerSec, HardwareMap hw){
        hardwareMap = hw;
        try {
            motors[MOTOR_1] = new MotorController("", "", hardwareMap);
            motors[MOTOR_2] = new MotorController("", "", hardwareMap);
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException(e.toString());
        }
        motors[MOTOR_1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[MOTOR_2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendSpeedInPerSecond = extendInPerSec;
        retractSpeedInPerSecond = retractInPerSec;
        motorStartTicks[MOTOR_1] = motors[MOTOR_1].getCurrentTick();
        motorStartTicks[MOTOR_2] = motors[MOTOR_2].getCurrentTick();
    }

    public void setDirection(int motor, DcMotor.Direction direction){
        motors[motor].setDirection(direction);
    }

    public void holdPosition(){
        motors[MOTOR_1].holdPosition();
        motors[MOTOR_2].holdPosition();
    }

    public void setZeroPowerBehavior(int motor, DcMotor.ZeroPowerBehavior b){
        motors[motor].setZeroPowerBehavior(b);
    }

    public void extend() { motors[MOTOR_1].setMotorPower(extendPower); motors[MOTOR_2].setMotorPower(extendPower); }
    public void retract() { motors[MOTOR_1].setMotorPower(-extendPower); motors[MOTOR_2].setMotorPower(-extendPower);}

    public void setPostitionInches(int motor, double positionInInches) { motors[motor].setPositionInches(positionInInches);}

    public void setPostitionTicks(int motor, int ticks) { motors[motor].setPositionTicks(ticks); }

    public long getPosition(int motor){
        return motors[motor].getCurrentTick();
    }

    public double getPositionInches(int motor) {
        return motors[motor].getInchesFromStart();
    }

    public void setMode(int motor, DcMotor.RunMode mode){
        motors[motor].setMode(mode);
    }

    public DcMotor.RunMode getMotorControllerMode(int motor) {
        return motors[motor].getMotorRunMode();
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
        motors[MOTOR_1].setInchesPerSecondVelocity(0);
        motors[MOTOR_2].setInchesPerSecondVelocity(0);
        motors[MOTOR_1].killMotorController();
        motors[MOTOR_2].killMotorController();
    }
}