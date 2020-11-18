package Actions.Ultimate;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;

import MotorControllers.MotorController;

/**
 * Author: Ethan Fisher
 * Date: 10/21/2020
 *
 * Used for grabbing and releasing the wobble goal
 */
public class WobbleGrabberV1 extends Thread {

    // TODO test grabbing

    public Servo claw;
    public MotorController arm;
    //private RevColorSensorV3 redSensor;

    private static final double ARM_POWER_DOWN = -0.5;
    private static final double ARM_POWER_UP = 0.5;

    private static final long ARM_TICKS_PER_SECOND = 300;

    private static final int ARM_DOWN_TICKS = 240;
    private static final int ARM_UP_TICKS = 0;

    public static final double CLAW_GRAB_ANGLE = 0.0;
    public static final double CLAW_RELEASE_ANGLE = 0.5;
    public static final double ANGLE_INCREMENT = 25;
    public static final double LOWERED_ANGLE = -170;
    public static final double RAISED_ANGLE = -25;

    public static final int PIXELS_FOR_WOBBLE_GRAB = 0;

    public boolean wobbleGrabbed;

    public WobbleGrabberV1(HardwareMap hardwareMap) throws Exception {
        claw = hardwareMap.servo.get("wobbleGrabberClaw");

        // don't worry about this line. Just test what's currently in the file
        arm = new MotorController("wobbleGrabberArm", "ActionConfig/WobbleArmConfig.json", hardwareMap);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //redSensor = hardwareMap.get(RevColorSensorV3.class, "redSensor");

        wobbleGrabbed = false;
    }

    public void lowerArm() {
//        arm.setTicksPerSecondVelocity(ARM_TICKS_PER_SECOND);
        arm.setPositionDegrees(LOWERED_ANGLE, ARM_POWER_DOWN); // Brooks said motor speed was too fast, thus the 3/4 power but can be adjusted later
    }

    public void raiseArm() {
//        arm.setTicksPerSecondVelocity(ARM_TICKS_PER_SECOND);
        arm.setPositionDegrees(RAISED_ANGLE, ARM_POWER_UP); // Brooks said motor speed was too fast, thus the 3/4 power but can be adjusted later
    }

    //    public void toggleArmPosition() {
//       arm.setTicksPerSecondVelocity(ARM_TICKS_PER_SECOND);
//       arm.setPositionDegrees(LOWERED_ANGLE, (3*ARM_POWER_DOWN)/4); // Brooks said motor speed was too fast, thus the 3/4 power but can be adjusted later
//        if (arm.getDegree() == 150) {
//
//        }
//
//        else if(arm.getDegree() == )
//    }


    public void decreaseAngle(){
        arm.setPositionDegrees(arm.getDegree() + ANGLE_INCREMENT, ARM_POWER_DOWN);
    }
    public void addAngle() {
        arm.setPositionDegrees(arm.getDegree() - ANGLE_INCREMENT, ARM_POWER_UP);
    }

    public void grabWobbleGoal() { claw.setPosition(CLAW_GRAB_ANGLE); }
    public void releaseWobbleGoal() { claw.setPosition(CLAW_RELEASE_ANGLE); }


    public void grabOrReleaseWobbleGoal() {
        if (wobbleGrabbed) releaseWobbleGoal();
        else grabWobbleGoal();
        wobbleGrabbed = !wobbleGrabbed;
    }

    public boolean armIsBusy() {
        return arm.isBusy();
    }

//    public int getRedPixels() { return redSensor.red(); }

}