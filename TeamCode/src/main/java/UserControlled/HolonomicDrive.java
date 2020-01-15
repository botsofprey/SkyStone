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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import Actions.StoneStackingSystemV2;
import DriveEngine.HolonomicDriveSystemTesting;
import SensorHandlers.LIDARSensor;
import SensorHandlers.LimitSwitch;
import SensorHandlers.SensorPackage;

@TeleOp(name="Annie V1", group="Competition")
@Disabled
public class HolonomicDrive extends LinearOpMode {
    // create objects and locally global variables here

    DcMotor frontLeft, frontRight, backLeft, backRight;


    @Override
    public void runOpMode() {
        // initialize objects and variables here
        // also create and initialize function local variables here
        frontLeft = hardwareMap.dcMotor.get("flMotor");
        frontRight = hardwareMap.dcMotor.get("frMotor");
        backLeft = hardwareMap.dcMotor.get("blMotor");
        backRight = hardwareMap.dcMotor.get("brMotor");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);


        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // nothing goes between the above and below lines
        waitForStart();
        // should only be used for a time keeper or other small things, avoid using this space when possible
        while (opModeIsActive()) {
            // main code goes here
            if (gamepad1.right_stick_x < 0) {
                frontLeft.setPower(-gamepad1.right_stick_x);
                frontRight.setPower(gamepad1.right_stick_x);
                backLeft.setPower(gamepad1.right_stick_x);
                backRight.setPower(-gamepad1.right_stick_x);

            } else if (gamepad1.right_stick_x > 0) {
                frontLeft.setPower(-gamepad1.right_stick_x);
                frontRight.setPower(gamepad1.right_stick_x);
                backLeft.setPower(gamepad1.right_stick_x);
                backRight.setPower(-gamepad1.right_stick_x);

            } else if (gamepad1.left_stick_x > 0) {
                frontLeft.setPower(-(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) * Math.sin(Math.toRadians(45))));
                frontRight.setPower(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) * Math.cos(Math.toRadians(45)));
                backLeft.setPower(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) * Math.sin(Math.toRadians(45)));
                backRight.setPower(-(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) * Math.cos(Math.toRadians(45))));

            } else if (gamepad1.left_stick_x < 0) {
                frontLeft.setPower((Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) * Math.sin(Math.toRadians(45))));
                frontRight.setPower(-(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) * Math.cos(Math.toRadians(45))));
                backLeft.setPower(-(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) * Math.sin(Math.toRadians(45))));
                backRight.setPower(Math.atan2(gamepad1.left_stick_y,  gamepad1.left_stick_x) * Math.cos(Math.toRadians(45)));

            } else if (gamepad1.left_stick_y > 0) {
                frontLeft.setPower(-(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) * Math.sin(Math.toRadians(45))));
                frontRight.setPower(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) * Math.cos(Math.toRadians(45)));
                backLeft.setPower(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) * Math.sin(Math.toRadians(45)));
                backRight.setPower(-(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) * Math.cos(Math.toRadians(45))));

            } else if (gamepad1.left_stick_y < 0) {
                frontLeft.setPower(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) * Math.sin(Math.toRadians(45)));
                frontRight.setPower(-(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) * Math.cos(Math.toRadians(45))));
                backLeft.setPower(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + Math.sin(Math.toRadians(45)));
                backRight.setPower(-(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) * Math.cos(Math.toRadians(45))));

            }

            telemetry.update();
        }

        // disable/kill/stop objects here
    }

}