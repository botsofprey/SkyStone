/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package UserControlled.Ultimate;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import Actions.Ultimate.RingIntakeSystemV1;
import Actions.Ultimate.ShooterSystemV1;
import Actions.Ultimate.WobbleGrabberV1;
import Actions.WobbleGrabberCaidenTest;
import Autonomous.ColorDetector;
import Autonomous.Location;
import Autonomous.VuforiaHelper;
import DriveEngine.Ultimate.UltimateNavigation;
import SensorHandlers.LimitSwitch;
import SensorHandlers.MagneticLimitSwitch;
import UserControlled.GamepadController;
import UserControlled.JoystickHandler;

/**
 * Author: Software Team 2020-2021
 *
 * Controls the Ultimate Goal Robot
 *
 * -------------- TLDR ---------------
 * Player One:
 *      joysticks - drive base
 *      start - slow mode
 *      a - shoot
 *      dpad up / down - raise / lower shooter angle
 *      dpad left / right - bring shooter all the way down / up
 *      right bumper - raise elevator
 *      left bumper - lower elevator
 *
 * Player Two:
 *      b - intake direction
 *      a - intake power
 *      dpad up / down - raise / lower wobble grabber arm
 *      x - resets arm position for grabbing wobble goal
 *      y - grabbing or releasing wobble goal
 */

@TeleOp(name="Ultimate V1", group="Competition")
//@Disabled
public class UltimateV1 extends LinearOpMode {

    // TODO add speed values and angles when using the wobble grabber

    // create objects and locally global variables here
    UltimateNavigation robot;
    JoystickHandler leftStick, rightStick;
    GamepadController controllerOne, controllerTwo;

    RingIntakeSystemV1 intake;
    ShooterSystemV1 shooter;
    WobbleGrabberV1 grabber;

//    ColorDetector redDetector;

    boolean eStop = false, slowMode = false;

    @Override
    public void runOpMode() {
        // initialize objects and variables here
        // also create and initialize function local variables here

        // initialize robot
        // TODO get starting angle
        try {
            robot = new UltimateNavigation(hardwareMap, new Location(0, 0, 270), "RobotConfig/UltimateV1.json");
        } catch (Exception e) {
            telemetry.addData("Robot Error", e.toString());
            telemetry.update();
        }

        // initialize systems
        try {
            intake = new RingIntakeSystemV1(hardwareMap);
            shooter = new ShooterSystemV1(hardwareMap);
            grabber = new WobbleGrabberV1(hardwareMap);
        } catch (Exception e) {
            telemetry.addData("Systems Error", e.toString());
            telemetry.update();
        }

        // initialize red detector
//        redDetector = new ColorDetector(new VuforiaHelper(hardwareMap), 0xFF, 0x00, 0x00, 0x22);

        // initialize joysticks
        leftStick = new JoystickHandler(gamepad1, JoystickHandler.LEFT_JOYSTICK);
        rightStick = new JoystickHandler(gamepad1, JoystickHandler.RIGHT_JOYSTICK);

        // initialize controllers
        controllerOne = new GamepadController(gamepad1);
        controllerTwo = new GamepadController(gamepad2);

        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // nothing goes between the above and below lines

        waitForStart();

        // puts the pinball servo on the outside
        shooter.pinballServo.setPosition(ShooterSystemV1.PINBALL_REST);

        // should only be used for a time keeper or other small things, avoid using this space when possible
        while (opModeIsActive()) {
            // main code goes here

            updateEStop();
            if (!eStop) {

                // start button controls slow mode
                if (controllerOne.startPressed())
                    slowMode = !slowMode;

                updateEStop();
                controlDrive();

                updateEStop();

                if (!eStop) {
                    controllerOne.update(gamepad1);
                    controllerTwo.update(gamepad2);

                    playerOneFunctions();
                    playerTwoFunctions();
                }

                controlMiscFunctions();
            }

            if (eStop)
                stopActions();

            // telemetry and logging data goes here
            telemetry.update();
        }

        // disable/kill/stop objects here
        robot.stopNavigation();
    }

    // misc functions here
    private void updateEStop() {
        if ((controllerOne.dpadDownHeld() && gamepad1.back) || (controllerTwo.dpadDownHeld() && gamepad2.back))
            eStop = !eStop;
    }

    private void controlDrive() {
        double drivePower = slowMode ? leftStick.magnitude() / 3 : leftStick.magnitude();
        double turnPower = slowMode ? rightStick.x() / 4 : rightStick.x();
        if (!eStop)
            robot.driveOnHeadingWithTurning(leftStick.angle(), drivePower, turnPower);
    }

    private void playerOneFunctions() {

        if (controllerOne.aPressed())
            shooter.shoot();

        if (controllerOne.bPressed())
            shooter.toggleWheelPower();

        if (controllerOne.dpadUpPressed())
            shooter.raiseShooter(0.05);

        if (controllerOne.dpadDownPressed())
            shooter.lowerShooter(0.05);

        if (controllerOne.dpadRightPressed())
            shooter.setShooter(0);

        if (controllerOne.rightBumperPressed())
            shooter.raiseElevator(this);

        if (controllerOne.leftBumperPressed())
            shooter.lowerElevator(this);

        if (controllerOne.dpadLeftPressed())
            shooter.setShooter(1);
            // todo : test (should set shooter angle to max)
        if (controllerOne.dpadRightPressed())
            shooter.setShooter(0);
            // todo : test (set shooter angle to minimum)
    }

    private void playerTwoFunctions() {

        if (controllerTwo.xPressed())
            grabber.lowerArm();

        if (controllerTwo.yPressed())
            grabber.grabOrReleaseWobbleGoal();

        if (controllerTwo.dpadUpPressed())
            grabber.addAngle();

        if (controllerTwo.dpadDownPressed())
            grabber.addAngle();

        if (controllerTwo.bPressed())
            intake.toggleIntakeDirection();

        if (controllerTwo.aPressed())
            intake.toggleIntakePower();

    }

    private void controlMiscFunctions() {
        shooter.update(this);
    }

    private void stopActions() {
        robot.brake();
    }
}
