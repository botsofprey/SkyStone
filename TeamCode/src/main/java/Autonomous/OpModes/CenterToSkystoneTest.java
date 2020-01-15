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

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Autonomous.Location;
import java.util.ArrayList;

import Autonomous.ImageProcessing.SkystoneImageProcessor;
import Autonomous.VuforiaHelper;
import DriveEngine.JennyNavigation;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

@Autonomous(name="CenterToSkystone Test", group="Testers")
//@Disabled
public class CenterToSkystoneTest extends LinearOpMode {
    // create objects and locally global variables here
    SkystoneImageProcessor stoneFinder;
    VuforiaHelper vuforia;
    JennyNavigation robot;
    DistanceSensor back, right, left;

    @Override
    public void runOpMode() {

        // initialize objects and variables here
        vuforia = new VuforiaHelper(hardwareMap);
        back = hardwareMap.get(DistanceSensor.class, "back");
        right = hardwareMap.get(DistanceSensor.class, "right");
        left = hardwareMap.get(DistanceSensor.class, "left");
        stoneFinder = new SkystoneImageProcessor(SkystoneImageProcessor.DESIRED_HEIGHT, SkystoneImageProcessor.DESIRED_WIDTH,.1,1, SkystoneImageProcessor.STONE_COLOR.BLACK);
        try {
            robot = new JennyNavigation(hardwareMap, new Location(0, 0), 0, "RobotConfig/AnnieV1.json");
        } catch (Exception e) {
            e.printStackTrace();
        }
        // also create and initialize function local variables here

        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // nothing goes between the above and below lines
        waitForStart();

        // rough center...
        boolean skystoneFound = false;
        while (opModeIsActive() && right.getDistance(INCH) > 5.5) {
            if(!skystoneFound) robot.driveOnHeadingPID(JennyNavigation.RIGHT,10, this);  //NOTE: MOVING RIGHT
            Bitmap bmp = null;
            ArrayList<Integer> blockCenters;
            long timeStart = System.currentTimeMillis();
            while (bmp == null && System.currentTimeMillis() - timeStart < 50) {
                bmp = vuforia.getImage(SkystoneImageProcessor.DESIRED_WIDTH, SkystoneImageProcessor.DESIRED_HEIGHT);
            }
            if(bmp != null){
                blockCenters = stoneFinder.findColumns(bmp, false);

                if (blockCenters.size() > 0) {
//                telemetry.addData("Skystone", "" + blockCenters.get(0));
                    skystoneFound = true;
                    if(robot.centerOnSkystone((blockCenters.size() == 1)? blockCenters.get(0):blockCenters.get(1), 10, 30,this)) break; //Get left-most skystone
                }else skystoneFound = false;
            }
        }
        robot.brake();
        double distToWall = right.getDistance(INCH);

        if(distToWall > 42) { // to the left of the center

        } else if(distToWall > 35) { // to the right of the center -- should be checking against the actual center

        } else if(distToWall > 28) { // to the left of the center

        } else if(distToWall > 25) { // to the right of the center -- should be checking against the actual center

        } else if(distToWall > 23) { // to the left of the center

        } else if(distToWall > 20) { // to the right of the center -- should be checking against the actual center

        }


        while (opModeIsActive());
        robot.stopNavigation(); // ALWAYS kill everything at the end, leads to crashes of the app otherwise
    }
}
