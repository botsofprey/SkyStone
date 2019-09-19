package UserControlled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import Actions.RakeSystem;

@TeleOp(name="Rake Test", group="Linear Opmode")
//@Disabled
public class RakeTest extends LinearOpMode {

    RakeSystem rakeSystem;

    @Override
    public void runOpMode() {
        rakeSystem = new RakeSystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.a) rakeSystem.extendRake();
            else if(gamepad1.b) rakeSystem.retractRake();
            else rakeSystem.pauseRake();

            if(gamepad1.x) rakeSystem.closeArm();
            else if(gamepad1.y) rakeSystem.openArm();

            telemetry.addData("Phone Data", "Yay!");
            telemetry.update();
        }
    }
}
