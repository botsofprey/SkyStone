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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Actions.StoneStackingSystemV1;
import Autonomous.Location;
import Autonomous.VisionHelper;
import DriveEngine.JennyNavigation;
import SensorHandlers.LIDARSensor;
import SensorHandlers.SensorPackage;

@Autonomous(name="Sensor Package Test", group="Testers")
//@Disabled
public class SensorPackageTest extends LinearOpMode {
    // create objects and locally global variables here
    SensorPackage sensors;

    @Override
    public void runOpMode() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        sensors = new SensorPackage(new LIDARSensor(hardwareMap.get(DistanceSensor.class, "left"), 0,"left"), new LIDARSensor(hardwareMap.get(DistanceSensor.class, "back"),  1,"back"));

        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // nothing goes between the above and below lines
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Left Sensor", sensors.getSensor(LIDARSensor.class, 0));
            telemetry.addData("Left Dist", sensors.getSensor(LIDARSensor.class, 0).getDistance(DistanceUnit.INCH));
            telemetry.addData("Back Sensor", sensors.getSensor(LIDARSensor.class, 1));
            telemetry.addData("Back Dist", sensors.getSensor(LIDARSensor.class, 1).getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

        sensors.kill();

        // finish drive code and test
        // may be a good idea to square self against wall

    }
    // misc functions here
}
