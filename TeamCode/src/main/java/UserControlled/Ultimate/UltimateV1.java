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

import Actions.Ultimate.RingIntakeSystemV1;
import Actions.Ultimate.ShooterSystemV1;
import Actions.Ultimate.WobbleGrabberV1;
import Actions.WobbleGrabberCaidenTest;
import Autonomous.Location;
import DriveEngine.Ultimate.UltimateNavigation;
import UserControlled.GamepadController;
import UserControlled.JoystickHandler;

/**
 * Author: Software Team 2020-2021
 *
 * Controls the Ultimate Goal Robot
 *
 * -------------- TLDR ---------------
 * Player One:
 *      a -
 *
 * Player Two:
 *      b -
 */

@TeleOp(name="Ultimate V1", group="Competition")
//@Disabled
public class UltimateV1 extends LinearOpMode {

    // create objects and locally global variables here
    UltimateNavigation robot;
    JoystickHandler leftStick, rightStick;
    GamepadController controllerOne, controllerTwo;

    RingIntakeSystemV1 intake;
    ShooterSystemV1 shooter;
    WobbleGrabberV1 grabber;

    boolean eStop = false, slowMode = false;

    @Override
    public void runOpMode() {
        // initialize objects and variables here
        // also create and initialize function local variables here

        // initialize robot
        try {
            robot = new UltimateNavigation(hardwareMap, new Location(0, 0, 270), "RobotConfig/UltimateV1.json");
        } catch (Exception e) {
            telemetry.addData("Robot Error", e.toString());
            telemetry.update();
        }

        if(robot == null)
            telemetry.addData("Robot is null!","");

        // initialize systems
        intake = new RingIntakeSystemV1(hardwareMap);
        shooter = new ShooterSystemV1(hardwareMap);
        grabber = new WobbleGrabberV1(hardwareMap);

//        sensors = new SensorPackage(new LIDARSensor(hardwareMap.get(DistanceSensor.class, "back"), "back"),
//                new LIDARSensor(hardwareMap.get(DistanceSensor.class, "left"), "left"),
//                new LIDARSensor(hardwareMap.get(DistanceSensor.class, "right"), "right"),
//                new LimitSwitch(hardwareMap.get(TouchSensor.class, "liftReset"), "liftReset"),
//                new LimitSwitch(hardwareMap.get(TouchSensor.class, "leftArmStop"), "leftArmStop"),
//                new LimitSwitch(hardwareMap.get(TouchSensor.class, "rightArmStop"), "rightArmStop"));

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
//        if ((controllerOne.dpadDownHeld() && gamepad1.back) || (controllerTwo.dpadDownHeld() && gamepad2.back))
//            eStop = !eStop;
    }

    private void controlDrive() {
        double drivePower = slowMode ? leftStick.magnitude() / 3 : leftStick.magnitude();
        double turnPower = slowMode ? rightStick.x() / 4 : rightStick.x();
        if (!eStop)
            robot.driveOnHeadingWithTurning(leftStick.angle(), drivePower, turnPower);
    }

    private void playerOneFunctions() {

        if (controllerOne.aPressed()) {
            telemetry.addData("A", "pressed");
            shooter.shoot();
        }

        // b toggles intake direction (setting it up or down)
        if (controllerOne.bPressed()) {
            telemetry.addData("B", "pressed");
            intake.toggleIntakeDirection();
        }

        if (controllerOne.yPressed()) {
            telemetry.addData("Y", "pressed");
            intake.toggleIntakePower();
        }

        // y grabs the wobble goal
//        if (controllerOne.yPressed())
//            grabber.grabWobbleGoal();
        // left trigger raises the hopper

        if (controllerOne.leftBumperPressed()) {
            telemetry.addData("Left bumper", "pressed");
            shooter.adjustHopperAngle();
        }

        // left bumper lowers the arm
        if (controllerOne.rightBumperPressed()) {
            telemetry.addData("Right bumper", "pressed");
            shooter.adjustShootingAngle();
        }

        if (controllerOne.dpadRightPressed()) {
            telemetry.addData("Wobble Grabbed", "pressed");
            grabber.grabWobbleGoal();
        }

        if (controllerOne.dpadLeftPressed()) {
            telemetry.addData("Wobble Released", "pressed");
            grabber.releaseWobbleGoal();
        }

    }

    private void playerTwoFunctions() {

        // left trigger raises the hopper
        if (gamepad2.left_trigger > 0.1)
            shooter.adjustHopperAngle();

        // left bumper lowers the arm
        if (controllerTwo.leftBumperPressed())
            shooter.adjustShootingAngle();

        // right trigger and bumper
//        if (controllerTwo.rightBumperPressed())
//            shooter.raiseArm();

    }

    private void stopActions() {
//        robot.brake();
    }
}
