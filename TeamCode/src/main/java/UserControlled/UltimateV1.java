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

package UserControlled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import Autonomous.Location;
import DriveEngine.AnnieNavigation;
import SensorHandlers.LIDARSensor;
import SensorHandlers.LimitSwitch;
import SensorHandlers.SensorPackage;

@TeleOp(name="Annie V1", group="Competition")
//@Disabled
public class UltimateV1 extends LinearOpMode {

    // create objects and locally global variables here
    AnnieNavigation robot;
    SensorPackage sensors;
    JoystickHandler leftStick, rightStick;
    boolean eStop = false, slowMode = false, tapeStopped = true, liftLowered = true, liftingToPos = false;
    boolean startReleased = true, eStopButtonsReleased = true, limitSwitchReleased = false,
            rightTrigger1Released = true, rightBumper1Released = true,
            p2DpadUpReleased = true, p2DpadRightReleased = true, p2DpadLeftReleased = true;
    int stonePosition = 0, blenderFeetPos = 0;

    @Override
    public void runOpMode() {
        // initialize objects and variables here
        // also create and initialize function local variables here
        try {
            robot = new AnnieNavigation(hardwareMap, new Location(0, 0), 0, "RobotConfig/AnnieV1.json");
        } catch (Exception e) {
            e.printStackTrace();
        }

        sensors = new SensorPackage(new LIDARSensor(hardwareMap.get(DistanceSensor.class, "back"), "back"),
                new LIDARSensor(hardwareMap.get(DistanceSensor.class, "left"), "left"),
                new LIDARSensor(hardwareMap.get(DistanceSensor.class, "right"), "right"),
                new LimitSwitch(hardwareMap.get(TouchSensor.class, "liftReset"), "liftReset"),
                new LimitSwitch(hardwareMap.get(TouchSensor.class, "leftArmStop"), "leftArmStop"),
                new LimitSwitch(hardwareMap.get(TouchSensor.class, "rightArmStop"), "rightArmStop"));

        leftStick = new JoystickHandler(gamepad1, JoystickHandler.LEFT_JOYSTICK);
        rightStick = new JoystickHandler(gamepad1, JoystickHandler.RIGHT_JOYSTICK);

        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // nothing goes between the above and below lines
        waitForStart();
        // should only be used for a time keeper or other small things, avoid using this space when possible
        long startTime = System.currentTimeMillis();
//        otherActions.retractTape();
        while (opModeIsActive()) {
            // main code goes here
//            if(System.currentTimeMillis() - startTime > 3500 && !tapeStopped) {
//                otherActions.pauseTape();
//                tapeStopped = true;
//            }

            updateEStop();
            if(!eStop) {
                if (startReleased && gamepad1.start) {
                    startReleased = false;
                    slowMode = !slowMode;
                } else if (!gamepad1.start) {
                    startReleased = true;
                }

                updateEStop();
                controlDrive();

                updateEStop();
                controlStoneStackingSystem();
            }
            if(eStop) {
                stopActions();
            }
            // telemetry and logging data goes here
            telemetry.update();
        }
        // disable/kill/stop objects here
        robot.stopNavigation();
    }

    // misc functions here
    private void updateEStop() {
        if(eStopButtonsReleased && ((gamepad1.dpad_down && gamepad1.back) || (gamepad2.dpad_down && gamepad2.back))) {
            eStopButtonsReleased = false;
            eStop = !eStop;
        } else if(!((gamepad1.dpad_down && gamepad1.back) || (gamepad2.dpad_down && gamepad2.back))) {
            eStopButtonsReleased = true;
        }
    }

    private void controlDrive() {
        double drivePower = slowMode ? leftStick.magnitude() / 3 : leftStick.magnitude();
        double turnPower = slowMode ? rightStick.x() / 4 : rightStick.x();
        if (!eStop)
            robot.driveOnHeadingWithTurning(leftStick.angle(), drivePower, turnPower);
    }

    void controlStoneStackingSystem() {
        if(!eStop) {

            //PLAYER 1

            if (gamepad1.a) {}
            else if (gamepad1.b) {}

            if (gamepad1.right_trigger > 0.1 && rightTrigger1Released) {
                rightTrigger1Released = false;
            } else if (!(gamepad1.right_trigger > 0.1) && !rightTrigger1Released) {
                rightTrigger1Released = true;
            }
            if (gamepad1.right_bumper && rightBumper1Released) {
                rightBumper1Released = false;
            } else if (!gamepad1.right_bumper && !rightBumper1Released) {
                rightBumper1Released = true;
            }

            if(gamepad1.left_trigger > 0.1) {}
            else if(gamepad1.left_bumper) {}

            // PLAYER 2

//            if(gamepad2.b) {
//                sss.releaseStoneCenter();
//                sleep(200);
//                robot.driveDistance(5, 180, 50, this);
//                liftLowered = false;
//                sss.lowerStones();
//            }
            if(!liftLowered && sensors.getSensor(LimitSwitch.class, "liftReset").isPressed()) {
            }

            if(gamepad2.dpad_up && p2DpadUpReleased && !gamepad1.right_bumper && !(gamepad1.right_trigger > 0.1)) {
                p2DpadUpReleased = false;
            } else if(!gamepad2.dpad_up && !p2DpadUpReleased) {
                p2DpadUpReleased = true;
            }
//            if(gamepad2.dpad_down && !gamepad1.right_bumper && !(gamepad1.right_trigger > 0.1)) {
//                liftLowered = false;
//                sss.lowerStones();
//                liftingToPos = true;
//                stonePosition = 0;
//            }
            if(gamepad2.right_trigger > 0.1) {
                liftingToPos = false;
            }
            else if(gamepad2.right_bumper && !sensors.getSensor(LimitSwitch.class, "liftReset").isPressed()) {
                liftingToPos = false;
            }

            if(gamepad2.left_trigger > 0.1) {}
            else if(gamepad2.left_bumper) {}
            else if(tapeStopped) {}

            if(-gamepad2.left_stick_y > 0.1) {}
            else if(-gamepad2.left_stick_y < -0.1 && !sensors.getSensor(LimitSwitch.class, "leftArmStop").isPressed()) {}

            if(-gamepad2.right_stick_y > 0.1) {}
            else if(-gamepad2.right_stick_y < -0.1 && !sensors.getSensor(LimitSwitch.class, "rightArmStop").isPressed()) {}
            else {}

            if(gamepad2.dpad_right && p2DpadRightReleased) {
                p2DpadRightReleased = false;
            } else if(!gamepad2.dpad_right && !p2DpadRightReleased) {
                p2DpadRightReleased = true;
                blenderFeetPos++;
            }
            if(gamepad2.dpad_left && p2DpadLeftReleased) {
                p2DpadLeftReleased = false;
            } else if(!gamepad2.dpad_left && !p2DpadLeftReleased) {
                p2DpadLeftReleased = true;
                blenderFeetPos--;
            }
            blenderFeetPos %= 3;
//            switch (blenderFeetPos) {
//                case 0:
//                    sss.setBlenderFeetDegrees(StoneStackingSystemV2.LEFT_FOOT_STORED, StoneStackingSystemV2.RIGHT_FOOT_STORED);
//                    break;
//                case 1:
//                    sss.releaseStoneWithBlenderFeet();
//                    break;
//                case 2:
//                    sss.grabStoneWithBlenderFeet();
//                    break;
//            }

            if (gamepad2.a) {}
            else if (gamepad2.y) {}
            // check if limit switch is pressed and reset the lift encoder
            if (sensors.getSensor(LimitSwitch.class, "liftReset").isPressed() && limitSwitchReleased) {
                limitSwitchReleased = false;
            } else if (!sensors.getSensor(LimitSwitch.class, "liftReset").isPressed() && !limitSwitchReleased) {
                limitSwitchReleased = true;
            }
        }
    }

    private void stopActions() {
        robot.brake();
    }
}
