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

package Autonomous.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import DriveEngine.HolonomicDriveSystemTesting;

import static DriveEngine.HolonomicDriveSystemTesting.BACK_LEFT_HOLONOMIC_DRIVE_MOTOR;
import static DriveEngine.HolonomicDriveSystemTesting.BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR;
import static DriveEngine.HolonomicDriveSystemTesting.FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR;
import static DriveEngine.HolonomicDriveSystemTesting.FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Encoder Tester", group="Testers")
//@Disabled
public class EncoderTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        HolonomicDriveSystemTesting driveBase = new HolonomicDriveSystemTesting(hardwareMap,"RobotConfig/RosannaV4.json");


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.dpad_up) driveBase.driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].setMotorPower(1);
            else driveBase.driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].setMotorPower(0);
            if(gamepad1.dpad_down) driveBase.driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].setMotorPower(1);
            else driveBase.driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].setMotorPower(0);
            if(gamepad1.dpad_left) driveBase.driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].setMotorPower(1);
            else driveBase.driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].setMotorPower(0);
            if(gamepad1.dpad_right) driveBase.driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].setMotorPower(1);
            else driveBase.driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].setMotorPower(0);

            for(int i = 0; i < driveBase.driveMotors.length; i++){
                telemetry.addData("Motor" + i, driveBase.driveMotors[i].getCurrentTick());
            }
            telemetry.addData("0: FL, 1: FR ", " 2: BR, 3: BL");
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
        driveBase.kill();
    }
}
