package Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RingIntakeSystemV1 {
    private DcMotor intakeMotor;

    public RingIntakeSystemV1(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
    }

    public void startIntake(){ intakeMotor.setPower(1); }
    public void stopIntake(){ intakeMotor.setPower(0); }


}
