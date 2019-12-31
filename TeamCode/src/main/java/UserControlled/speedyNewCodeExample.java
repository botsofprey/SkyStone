package UserControlled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import DriveEngine.HolonomicDriveSystemTesting;

/**
 * Created by robotics on 2/16/18.
 */
@TeleOp(name="New Speedy Tester", group="Testers")  // @Autonomous(...) is the other common choice
@Disabled
public class speedyNewCodeExample extends LinearOpMode {

    final double movementScale = 1;
    final double turningScale = .5;

    @Override
    public void runOpMode() throws InterruptedException {
        HolonomicDriveSystemTesting driveSystem = new HolonomicDriveSystemTesting(hardwareMap,"RobotConfig/RosannaV4.json");
        JoystickHandler leftStick = new JoystickHandler(gamepad1, JoystickHandler.LEFT_JOYSTICK);
        JoystickHandler rightStick = new JoystickHandler(gamepad1, JoystickHandler.RIGHT_JOYSTICK);
        waitForStart();
        double movementPower = 0;
        double turningPower = 0;

        while(opModeIsActive()){
            movementPower = movementScale * Math.abs(leftStick.magnitude());
            turningPower = turningScale * Math.abs(rightStick.magnitude()) * Math.signum(rightStick.x());
            driveSystem.cartesianDriveOnHeadingWithTurning(leftStick.angle(), movementPower, turningPower);
            telemetry.addData("Gamepad1 left Joystick",leftStick.toString());
            telemetry.addData("Gamepad1 right Joystick", rightStick.toString());
            telemetry.addData("Robot Heading", driveSystem.orientation.getOrientation());
            telemetry.update();
        }
        driveSystem.kill();
    }

}
