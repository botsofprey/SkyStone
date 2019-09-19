package UserControlled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Claw Test", group="Linear Opmode")
//@Disabled
public class ClawTest extends LinearOpMode {

    Servo claw;

    @Override
    public void runOpMode() {
        claw = hardwareMap.servo.get("claw");
        claw.setDirection(Servo.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.x) claw.setPosition(0);
            else if(gamepad1.y) claw.setPosition(0.5);

            telemetry.addData("Phone Data", "Yay!");
            telemetry.update();
        }
    }
}
