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

import Actions.MiscellaneousActions;
import Actions.StoneStackingSystemV2;
import Autonomous.ImageProcessing.SkystoneImageProcessor;
import Autonomous.Location;
import Autonomous.VuforiaHelper;
import DriveEngine.JennyNavigation;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

@Autonomous(name="BlueV2", group="Competition")
//@Disabled
public class AnnieV2AutoRed extends LinearOpMode {
    // create objects and locally global variables here
    JennyNavigation robot;
    StoneStackingSystemV2 sss;
    VuforiaHelper vuforia;
    SkystoneImageProcessor stoneFinder;
    DistanceSensor back, right, left;
    MiscellaneousActions otherActions;

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
        otherActions = new MiscellaneousActions(hardwareMap);

        try {
            robot = new JennyNavigation(hardwareMap, new Location(0, 0), 0, "RobotConfig/AnnieV1.json");
        } catch (Exception e) {
            e.printStackTrace();
        }
        sss = new StoneStackingSystemV2(hardwareMap);

        // Stone detection


        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // nothing goes between the above and below lines
        waitForStart();

        if(opModeIsActive()) sss.setCentralGripperDegree(StoneStackingSystemV2.CENTRAL_ARM_GRAB);
        sleep(500);
        if(opModeIsActive()) sss.setCentralGripperDegree(StoneStackingSystemV2.CENTRAL_ARM_RELEASE);

        robot.driveDistance(7, JennyNavigation.FORWARD, 15, this);

//        telemetry.addData("Left", Double.toString(left.getDistance(INCH)));
//        telemetry.addData("Right", Double.toString(right.getDistance(INCH)));
//        telemetry.addData("Back", Double.toString(back.getDistance(INCH)));
//        telemetry.update();

//        Drive forward until 15 inches from wall
//        double distToWall = back.getDistance(INCH);
//        while (opModeIsActive() && distToWall < 15) {
//            robot.driveOnHeadingPID(0, 10, 0, this);
//            distToWall = back.getDistance(INCH);
//
//            telemetry.addData("Left", Double.toString(left.getDistance(INCH)));
//            telemetry.addData("Right", Double.toString(right.getDistance(INCH)));
//            telemetry.addData("Back", Double.toString(back.getDistance(INCH)));
//            telemetry.update();
//        }
//        robot.brake();

//        Search for skystone and break if found
        boolean skystoneFound = false;
        while (opModeIsActive() && left.getDistance(INCH) > 5.5) {
            if(!skystoneFound) robot.driveOnHeadingPID(JennyNavigation.LEFT,10, this);  //NOTE: MOVING RIGHT
            Bitmap bmp = null;
            ArrayList<Integer> blockCenters;
            while (bmp == null) {
                bmp = vuforia.getImage(SkystoneImageProcessor.DESIRED_WIDTH, SkystoneImageProcessor.DESIRED_HEIGHT);
            }
            blockCenters = stoneFinder.findColumns(bmp, false);

            if (blockCenters.size() > 0) {
//                telemetry.addData("Skystone", "" + blockCenters.get(0));
                skystoneFound = true;
                if(robot.centerOnSkystone((blockCenters.size() == 1)? blockCenters.get(0):blockCenters.get(1), 0, 30,this)) break; //Get left-most skystone
            }else skystoneFound = false;

//            telemetry.addData("Left", Double.toString(left.getDistance(INCH)));
//            telemetry.addData("Right", Double.toString(right.getDistance(INCH)));
//            telemetry.addData("Back", Double.toString(back.getDistance(INCH)));
//            telemetry.update();
        }
        robot.brake();

//        Grab skystone
        while(opModeIsActive() && back.getDistance(INCH) < 27.5) {
            robot.driveOnHeadingPID(JennyNavigation.FORWARD,15,0,this);
//            telemetry.addData("Left", Double.toString(left.getDistance(INCH)));
//            telemetry.addData("Right", Double.toString(right.getDistance(INCH)));
//            telemetry.addData("Back", Double.toString(back.getDistance(INCH)));
//            telemetry.update();
        }
        robot.brake();

        sss.grabStoneCenter();
        sleep(750); // wait to grab the stone TODO: grabStoneCenterAndWait() to wait until correct angle reached?
        while(opModeIsActive() && back.getDistance(INCH) > 25) robot.driveOnHeadingPID(180, 20, 0, this);
        robot.brake();

//        Drive to foundation
        robot.driveDistance(42,JennyNavigation.RIGHT,35,this);
        if(opModeIsActive()) sss.liftToPosition(1);
        while (opModeIsActive() && right.getDistance(INCH) > 30) {
            robot.driveOnHeadingPID(JennyNavigation.RIGHT, 30, 0, this);

//            telemetry.addData("Left", Double.toString(left.getDistance(INCH)));
//            telemetry.addData("Right", Double.toString(right.getDistance(INCH)));
//            telemetry.addData("Back", Double.toString(back.getDistance(INCH)));
//            telemetry.update();
        }
        robot.brake();

        // lift stone and stack


        // drive distance
        for(int i = 0; i < 2; i++) { //Double check
            while (opModeIsActive() && back.getDistance(INCH) < 27.5){
                robot.driveOnHeadingPID(0, 15, 0, this);

//                telemetry.addData("Left", Double.toString(left.getDistance(INCH)));
//                telemetry.addData("Right", Double.toString(right.getDistance(INCH)));
//                telemetry.addData("Back", Double.toString(back.getDistance(INCH)));
//                telemetry.update();
            }
            robot.brake();
        }
        sleep(200);
//        if(opModeIsActive()) sss.liftToPosition(0);
//        while(opModeIsActive() && sss.getLiftPositionInches() < 2.9);
//        if(opModeIsActive()) sss.setLiftPosition(2.5);
//        while(opModeIsActive() && sss.getLiftPositionInches() > 2.55);

        if(opModeIsActive()) sss.releaseStoneCenter();

        while (opModeIsActive() && back.getDistance(INCH) > 26) {
            robot.driveOnHeadingPID(JennyNavigation.BACK, 15, 0, this);

//            telemetry.addData("Left", Double.toString(left.getDistance(INCH)));
//            telemetry.addData("Right", Double.toString(right.getDistance(INCH)));
//            telemetry.addData("Back", Double.toString(back.getDistance(INCH)));
//            telemetry.update();
        }
        robot.brake();

        while(opModeIsActive() && right.getDistance(INCH) > 48) robot.driveOnHeadingPID(JennyNavigation.RIGHT, 15, this);
        robot.brake();
//
//        robot.turnToHeading(90, this);
//
//        robot.brake();
//        sleep(500);
//
//        while (opModeIsActive() && right.getDistance(INCH) < 31){
//            robot.driveOnHeadingPID(JennyNavigation.LEFT, 15, 0, this);
//
////                telemetry.addData("Left", Double.toString(left.getDistance(INCH)));
//                telemetry.addData("Right", Double.toString(right.getDistance(INCH)));
////                telemetry.addData("Back", Double.toString(back.getDistance(INCH)));
//                telemetry.update();
//        }
//        robot.brake();
//        otherActions.grabFoundation();
//        sleep(500);
//        robot.driveOnHeadingWithTurning(90,.3,-.2);
//        sleep(1500);
//        while (opModeIsActive());
        otherActions.spitTape();
        sleep(1000);
        otherActions.pauseTape();
        robot.brake();
        robot.stopNavigation();
        sss.kill();
//        VuforiaHelper.kill(); -- this crashes the app...

        // finish drive code and test
        // may be a good idea to square self against wall

    }
    // misc functions here
}
