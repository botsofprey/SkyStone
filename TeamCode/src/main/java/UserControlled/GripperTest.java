package UserControlled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import Actions.StoneStackingSystem;

@TeleOp(name="Gripper Test", group="Linear Opmode")
//@Disabled
public class GripperTest extends LinearOpMode {
    StoneStackingSystem sss;

    @Override
    public void runOpMode() {
        sss = new StoneStackingSystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.right_trigger > 0.1) sss.liftStones();
            else if(gamepad1.right_bumper) sss.lowerStones();
            else sss.pauseLift();

            if (gamepad1.a) sss.grabStone();
            else if (gamepad1.b) sss.releaseStone();

            telemetry.addData("Phone Data", "Yay!");
            telemetry.update();
        }
    }
}
