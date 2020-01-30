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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.ArrayList;

import Actions.MiscellaneousActionsV2;
import Actions.StoneStackingSystemV3;
import Autonomous.ImageProcessing.SkystoneImageProcessor;
import Autonomous.Location;
import Autonomous.VuforiaHelper;
import DriveEngine.JennyNavigation;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

@Autonomous(name="RedV2", group="Competition")
//@Disabled
public class AnnieV2AutoRed extends LinearOpMode {
    // create objects and locally global variables here
    JennyNavigation robot;
    StoneStackingSystemV3 sss;
    VuforiaHelper vuforia;
    SkystoneImageProcessor stoneFinder;
    DistanceSensor back, right, left;
    MiscellaneousActionsV2 otherActions;

    @Override
    public void runOpMode() {
        // initialize objects and variables here
        // also create and initialize function local variables here
        vuforia = new VuforiaHelper(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        back = hardwareMap.get(DistanceSensor.class, "back");
        right = hardwareMap.get(DistanceSensor.class, "right");
        left = hardwareMap.get(DistanceSensor.class, "left");
        stoneFinder = new SkystoneImageProcessor(SkystoneImageProcessor.DESIRED_HEIGHT, SkystoneImageProcessor.DESIRED_WIDTH,.1,1, SkystoneImageProcessor.STONE_COLOR.BLACK);
        otherActions = new MiscellaneousActionsV2(hardwareMap);

        try {
            robot = new JennyNavigation(hardwareMap, new Location(0, 0), 0, "RobotConfig/AnnieV1.json");
        } catch (Exception e) {
            e.printStackTrace();
        }
        sss = new StoneStackingSystemV3(hardwareMap);

        // Stone detection


        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // nothing goes between the above and below lines
        waitForStart();

        if(opModeIsActive()) sss.setCentralGripperDegree(StoneStackingSystemV3.CENTRAL_ARM_GRAB);
        sleep(500);
        if(opModeIsActive()) sss.setCentralGripperDegree(StoneStackingSystemV3.CENTRAL_ARM_RELEASE);

        robot.driveDistance(7, JennyNavigation.FORWARD, 20, this);


        // NEW SKYSTONE SEARCHING ALGORITHM!!!
        // LOOK IF YOU SEE BLACK ON THE SCREEN, IF SO, GO TO THE PROPER LIDAR DISTANCE FROM THE WALL TO GET THAT SKYSTONE!!!

//        Search for skystone and break if found
        boolean skystoneFound = false;
        while (opModeIsActive() && left.getDistance(INCH) > 5.5) {
            if(!skystoneFound) robot.driveOnHeadingPID(JennyNavigation.LEFT,10, this);  //NOTE: MOVING RIGHT
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
                    if(robot.centerOnSkystone((blockCenters.size() == 1)? blockCenters.get(0):blockCenters.get(1), 3, 30,this)) break; //Get left-most skystone
                }else skystoneFound = false;
            }
        }
        robot.brake();

//      Grab skystone
        while (opModeIsActive() && back.getDistance(INCH) < 32) {
            robot.driveOnHeadingPID(JennyNavigation.FORWARD, 15, 0, this);
        }
        robot.brake();

        double distToWall = left.getDistance(INCH);
        if (distToWall > 37) { // good
            while (opModeIsActive() && left.getDistance(INCH) < 37) robot.driveOnHeadingPID(JennyNavigation.RIGHT, 20, this); // STONE 1 RIGHT
            telemetry.addData("first stone", "move right");
            telemetry.update();
        } else if (distToWall > 28.5) { // good
            while (opModeIsActive() && left.getDistance(INCH) > 38.50) robot.driveOnHeadingPID(JennyNavigation.LEFT, 20, this);// STONE 1 LEFT
            telemetry.addData("first stone", "move left");
            telemetry.update();
        } else if (distToWall > 21) { // good for now
            while (opModeIsActive() && left.getDistance(INCH) < 37.25) robot.driveOnHeadingPID(JennyNavigation.RIGHT, 20, this);// STONE 2 RIGHT
            telemetry.addData("second stone", "move right");
            telemetry.update();
        } else if (distToWall > 28) { // keep testing
            while (opModeIsActive() && left.getDistance(INCH) > 30.00) robot.driveOnHeadingPID(JennyNavigation.LEFT, 20, this);// STONE 2 LEFT
            telemetry.addData("second stone", "move left");
            telemetry.update();
        } else if (distToWall > 24) { // keep testing
            while (opModeIsActive() && left.getDistance(INCH) < 24.50) robot.driveOnHeadingPID(JennyNavigation.RIGHT, 20, this);// STONE 3 RIGHT
            telemetry.addData("third stone", "move right");
            telemetry.update();
        } else if (distToWall > 19.5) { // keep testing
            while (opModeIsActive() && left.getDistance(INCH) > 24.50) robot.driveOnHeadingPID(JennyNavigation.LEFT, 20, this);// STONE 3 LEFT
            telemetry.addData("third stone", "move left");
            telemetry.update();
        }
        robot.brake();

        sss.grabStoneCenter();
        sleep(750); // wait to grab the stone TODO: grabStoneCenterAndWait() to wait until correct angle reached?
        while(opModeIsActive() && back.getDistance(INCH) > 25) robot.driveOnHeadingPID(180, 20, 0, this);
        robot.brake();

//        Drive to foundation
        robot.driveDistance(42,JennyNavigation.RIGHT,35,this);

        while (opModeIsActive() && right.getDistance(INCH) > 31) {
            robot.driveOnHeadingPID(JennyNavigation.RIGHT, 30, 0, this);
        }
        sss.releaseStoneCenter();
        robot.brake();

        //Second stone
        while(opModeIsActive() && left.getDistance(INCH) > 24) {
            robot.driveOnHeadingPID(JennyNavigation.LEFT, 35, 0, this);
        }
        robot.brake();

        //DUPLICATE CODE
        if(distToWall > 35.4) { // first stone
            while(opModeIsActive() && left.getDistance(INCH) > 17.5) robot.driveOnHeadingPID(JennyNavigation.LEFT, 20, 0, this);
        } else if(distToWall > 28) { // second stone
            while(opModeIsActive() && left.getDistance(INCH) > 7) robot.driveOnHeadingPID(JennyNavigation.LEFT, 20, 0, this);
        } else if(distToWall > 19.5) { // third stone
            while(opModeIsActive() && left.getDistance(INCH) > 5) robot.driveOnHeadingPID(JennyNavigation.LEFT, 20, 0, this);
        }
        robot.brake();

//        Grab skystone
//        for(int i = 0; i < 3; i++) {
        while (opModeIsActive() && back.getDistance(INCH) < 32) {
            robot.driveOnHeadingPID(JennyNavigation.FORWARD, 15, 0, this);
        }
//        }
        robot.brake();

        sss.grabStoneCenter();
        sleep(750); // wait to grab the stone TODO: grabStoneCenterAndWait() to wait until correct angle reached?
        while(opModeIsActive() && back.getDistance(INCH) > 25) robot.driveOnHeadingPID(180, 20, 0, this);
        robot.brake();

//        Drive to foundation
        robot.driveDistance(42,JennyNavigation.RIGHT,35,this);

        while (opModeIsActive() && right.getDistance(INCH) > 31) {
            robot.driveOnHeadingPID(JennyNavigation.RIGHT, 30, 0, this);
        }
        sss.releaseStoneCenter();

        if(distToWall > 35) {
            // go to the left skystone
        } else if(distToWall > 25) {
            // go to the middle skystone
        } else if(distToWall > 20) {
            // go to the right skystone
        }

        while (opModeIsActive());
        robot.stopNavigation();
        sss.kill();


        //FOUNDATION
        while(opModeIsActive() && right.getDistance(INCH) > 48) robot.driveOnHeadingPID(JennyNavigation.RIGHT, 15, this);
        robot.brake();

//        robot.turnToHeading(180, this);
        robot.turnToHeading(90, 2,this);

        robot.brake();
        sleep(500);

        robot.turnController.setSp(robot.getOrientation());
        while (opModeIsActive() && left.getDistance(INCH) < 31){
            robot.driveOnHeadingPID(JennyNavigation.RIGHT, 15, 0, this);
//
////                telemetry.addData("Left", Double.toString(left.getDistance(INCH)));
//                telemetry.addData("Right", Double.toString(right.getDistance(INCH)));
////                telemetry.addData("Back", Double.toString(back.getDistance(INCH)));
//                telemetry.update();
        }

        robot.brake();
        otherActions.grabFoundation();
        sleep(500);
        robot.driveOnHeadingWithTurning(100,1,.4);
        sleep(1300);
//        while (opModeIsActive());
        otherActions.releaseFoundation();
        robot.driveDistance(15,0,25,this);
        robot.driveDistance(10, 90, 25, this);
        otherActions.spitTape();
        sleep(500);
        otherActions.pauseTape();
        robot.brake();
        robot.driveDistance(10, -90, 25, this);
        robot.stopNavigation();
        sss.kill();
//        VuforiaHelper.kill(); -- this crashes the app...

        // finish drive code and test
        // may be a good idea to square self against wall

    }
    // misc functions here
}
