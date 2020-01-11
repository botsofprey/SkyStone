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

package Autonomous.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import Actions.StoneStackingSystemV2;
import Autonomous.Location;
import Autonomous.VisionHelper;
import DriveEngine.JennyNavigation;

@Autonomous(name="PseudoBlueRight", group="Competition")
@Disabled

public class PseudocodeAutoBlueLeft extends LinearOpMode {
    // create objects and locally global variables here
    JennyNavigation robot;
    StoneStackingSystemV2 sss;
    VisionHelper vision;
    DistanceSensor back, right, left;

    @Override
    public void runOpMode() {
        // initialize objects and variables here
        // also create and initialize function local variables here
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        back = hardwareMap.get(DistanceSensor.class, "back");
        right = hardwareMap.get(DistanceSensor.class, "right");
        left = hardwareMap.get(DistanceSensor.class, "left");
        try {
            robot = new JennyNavigation(hardwareMap, new Location(0, 0), 0, "RobotConfig/AnnieV1.json");
        } catch (Exception e) {
            e.printStackTrace();
        }
        sss = new StoneStackingSystemV2(hardwareMap);

        // Stone detection
        vision = new VisionHelper(VisionHelper.WEBCAM, VisionHelper.LOCATION, hardwareMap);
        vision.startDetection();
        vision.startTrackingLocation();

        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        vision.startDetection();
        // nothing goes between the above and below lines
        waitForStart();

        //LOOP until skystone found OR second-to-last block distance away

        //Move right under bridge near searching start position
        //Move right
        //Take image
        //Check for skystone
        //IF skystone BREAK

        //Center to Skystone
        //Move forward
        //Grab with central gripper
        //Move backward
        //Move left until correct distance from wall
        //Move forward to foundation
        //Release with central gripper

        //Rotate to side with foundation grabber
        //Engage foundation grabber
        //Pull back until certain dist from wall
        //Rotate 90 degrees
        //Release foundation using foundation grabber
        //Move right some distance
        //Spit tape

        while(opModeIsActive());
        // may be a good idea to square self against wall

    }
    // misc functions here
}
