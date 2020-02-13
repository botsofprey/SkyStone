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

import Actions.MiscellaneousActionsV2;
import Actions.StoneStackingSystemV3;
import Autonomous.ImageProcessing.SkystoneImageProcessor;
import Autonomous.Location;
import Autonomous.VuforiaHelper;
import DriveEngine.AnnieNavigation;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

@Autonomous(name="Detect Skystone From Wall", group="Competition")
//@Disabled
public class DetectSkystoneFromWall extends LinearOpMode {
    // create objects and locally global variables here
    AnnieNavigation robot;
    StoneStackingSystemV3 sss;
    VuforiaHelper vuforia;
    SkystoneImageProcessor stoneFinder;
    DistanceSensor back, right, left;
    MiscellaneousActionsV2 otherActions;
    static final double LEFT_DIST = .48, BACK_DIST = .91, CENTER_DIST = .73, RIGHT_DIST = .91; //LEFT .54 LEAVE THIS COMMENT FOR NOW PLEASE
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
            robot = new AnnieNavigation(hardwareMap, new Location(0, 0), 0, "RobotConfig/AnnieV1.json");
        } catch (Exception e) {
            e.printStackTrace();
        }
        sss = new StoneStackingSystemV3(hardwareMap);

        vuforia.getImage(SkystoneImageProcessor.DESIRED_WIDTH, SkystoneImageProcessor.DESIRED_HEIGHT);
        sleep(100);
        // Stone detection

        while(!opModeIsActive()) {
            int pos = stoneFinder.getSkystoneRelativePosition(vuforia.getImage(SkystoneImageProcessor.DESIRED_WIDTH, SkystoneImageProcessor.DESIRED_HEIGHT));
            String posStr = "";
            switch (pos) {
                case SkystoneImageProcessor.LEFT:
                    posStr = "LEFT";
                    break;
                case SkystoneImageProcessor.CENTER:
                    posStr = "CENTER";
                    break;
                case SkystoneImageProcessor.RIGHT:
                    posStr = "RIGHT";
                    break;
                default:
                    posStr = "NOT FOUND";
                    break;
            }
            telemetry.addData("Skystone position", posStr);
            // add any other useful telemetry data or logging data here
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Op Mode", opModeIsActive());
            telemetry.update();
        }
        // nothing goes between the above and below lines
        waitForStart();

        //TODO: should we only drop if we found skystone?
        if(opModeIsActive()) sss.setCentralGripperDegree(StoneStackingSystemV3.CENTRAL_ARM_GRAB);
        sleep(500);
        if(opModeIsActive()) sss.setCentralGripperDegree(StoneStackingSystemV3.CENTRAL_ARM_RELEASE);

        // FOR RED
        // LEFT: .54 m
        // RIGHT: .91 m

        //TODO: Don't you need to initialize the turnController before using driveOnHeading? -- Mr. McDonald
        //Example: robot.turnController.setSp(robot.getOrientation());
//        if (posStr.equals("CENTER")) {
//            while (back.getDistance(METER) < .91 && opModeIsActive()) {
//                robot.driveOnHeadingPID(AnnieNavigation.FORWARD, 25, this);
//            }
//        } else if (posStr.equals("LEFT")) {
//            while (back.getDistance(METER) < .91 && left.getDistance(METER) > LEFT_DIST && opModeIsActive()) {
//                robot.driveOnHeadingPID(Math.toDegrees(Math.atan2(LEFT_DIST - CENTER_DIST, BACK_DIST)), 25, this);
//            }
//        } else if (posStr.equals("RIGHT")) {
//            while (back.getDistance(METER) < .91 && left.getDistance(METER) < RIGHT_DIST && opModeIsActive()) {
//                robot.driveOnHeadingPID(Math.toDegrees(Math.atan2(RIGHT_DIST - CENTER_DIST, BACK_DIST)), 25, this);
//            }
//        }
        robot.brake();
        //else check again
        telemetry.addData("Angle L: ", Math.atan2(LEFT_DIST - CENTER_DIST, BACK_DIST));
        telemetry.addData("Angle R: ", Math.atan2(RIGHT_DIST - CENTER_DIST, BACK_DIST));
        telemetry.update();
//        double distToWall = left.getDistance(INCH);
//        if (distToWall > STONE_ONE_LEFT) { //
//            while (opModeIsActive() && left.getDistance(INCH) > DIST_STONE_ONE_LEFT) robot.driveOnHeadingPID(AnnieNavigation.LEFT, 20, this); // STONE 1 LEFT
//            telemetry.addData("first stone", "move left");
//            telemetry.update();
//        } else if (distToWall > STONE_ONE_RIGHT) { //
//            while (opModeIsActive() && left.getDistance(INCH) < DIST_STONE_ONE_RIGHT) robot.driveOnHeadingPID(AnnieNavigation.RIGHT, 20, this);// STONE 1 RIGHT
//            telemetry.addData("first stone", "move right");
//            telemetry.update();
//        } else if (distToWall > STONE_TWO_RIGHT) { //
//            while (opModeIsActive() && left.getDistance(INCH) < DIST_STONE_TWO_RIGHT) robot.driveOnHeadingPID(AnnieNavigation.RIGHT, 20, this);// STONE 2 RIGHT
//            telemetry.addData("second stone", "move right");
//            telemetry.update();
//        } else if (distToWall > STONE_TWO_LEFT) { //
//            while (opModeIsActive() && left.getDistance(INCH) > DIST_STONE_TWO_LEFT) robot.driveOnHeadingPID(AnnieNavigation.LEFT, 20, this);// STONE 2 LEFT
//            telemetry.addData("second stone", "move left");
//            telemetry.update();
//        } else if (distToWall > STONE_THREE_LEFT) { //
//            while (opModeIsActive() && left.getDistance(INCH) > DIST_STONE_THREE_LEFT) robot.driveOnHeadingPID(AnnieNavigation.LEFT, 20, this);// STONE 3 LEFT
//            telemetry.addData("third stone", "move left");
//            telemetry.update();
//        } else if (distToWall > STONE_THREE_RIGHT) { //
//            while (opModeIsActive() && left.getDistance(INCH) < DIST_STONE_THREE_RIGHT) robot.driveOnHeadingPID(AnnieNavigation.RIGHT, 20, this);// STONE 3 RIGHT
//            telemetry.addData("third stone", "move right");
//            telemetry.update();
//        }
//        robot.brake();

        sss.grabStoneCenter();

//        sleep(750); // wait to grab the stone TODO: grabStoneCenterAndWait() to wait until correct angle reached?
//        while(opModeIsActive() && back.getDistance(INCH) > 25) robot.driveOnHeadingPID(180, 20, 0, this);
//        robot.brake();
//
////        Drive to foundation
//        robot.driveDistance(42, AnnieNavigation.RIGHT,35,this);
//
////        sss.liftToPosition(1); // For foundation dropping
//        while (opModeIsActive() && right.getDistance(INCH) > 31) {
//            robot.driveOnHeadingPID(AnnieNavigation.RIGHT, 30, 0, this);
//        }
////        while (opModeIsActive() && back.getDistance(INCH) > 38) {  // For foundation dropping
////            robot.driveOnHeadingPID(AnnieNavigation.FORWARD, 30, 0, this);
////        }
//
//        sss.releaseStoneCenter();
//        robot.brake();
//
//        //Second stone
//        while(opModeIsActive() && left.getDistance(INCH) > 24) {
//            robot.driveOnHeadingPID(AnnieNavigation.LEFT, 35, 0, this);
//        }
//        robot.brake();
//
//        //DUPLICATE CODE
////        if(distToWall > STONE_FOUR) { // first stone
////            while(opModeIsActive() && left.getDistance(INCH) > DIST_STONE_FOUR) robot.driveOnHeadingPID(AnnieNavigation.LEFT, 20, 0, this);
////        } else if(distToWall > STONE_FIVE) { // second stone
////            while(opModeIsActive() && left.getDistance(INCH) > DIST_STONE_FIVE) robot.driveOnHeadingPID(AnnieNavigation.LEFT, 20, 0, this);
////        } else if(distToWall > STONE_SIX) { // third stone
////            while(opModeIsActive() && left.getDistance(INCH) > 13.3) robot.driveOnHeadingPID(AnnieNavigation.LEFT, 20, 0, this);
////        }
////        robot.brake();
//
////        Grab skystone
////        for(int i = 0; i < 3; i++) {
//        while (opModeIsActive() && back.getDistance(INCH) < 36) { //Check this later
//            robot.driveOnHeadingPID(AnnieNavigation.FORWARD, 15, 0, this);
//        }
////        }
//        robot.brake();
//
//        sss.grabStoneCenter();
//        sleep(750); // wait to grab the stone TODO: grabStoneCenterAndWait() to wait until correct angle reached?
//        while(opModeIsActive() && back.getDistance(INCH) > 25) robot.driveOnHeadingPID(180, 20, 0, this);
//        robot.brake();
//
////        Drive to foundation
//        robot.driveDistance(42,AnnieNavigation.RIGHT,35,this);
//
//        while (opModeIsActive() && right.getDistance(INCH) > 31) {
//            robot.driveOnHeadingPID(AnnieNavigation.RIGHT, 30, 0, this);
//        }
//        sss.releaseStoneCenter();
//        robot.brake();

        while (opModeIsActive());
        robot.stopNavigation();
        sss.kill();


        //FOUNDATION
        while(opModeIsActive() && right.getDistance(INCH) > 48) robot.driveOnHeadingPID(AnnieNavigation.RIGHT, 15, this);
        robot.brake();

//        robot.turnToHeading(180, this);
        robot.turnToHeading(90, 2,this);

        robot.brake();
        sleep(500);

        robot.turnController.setSp(robot.getOrientation());
        while (opModeIsActive() && left.getDistance(INCH) < 31){
            robot.driveOnHeadingPID(AnnieNavigation.RIGHT, 15, 0, this);
//
////                telemetry.addData("Left", Double.toString(left.getDistance(INCH)));
//                telemetry.addData("Right", Double.toString(right.getDistance(INCH)));
////                telemetry.addData("Back", Double.toString(back.getDistance(INCH)));
//                telemetry.update();
        }

        robot.brake();
        robot.turnToHeading(180,5,this);
        sleep(1000);
        //SPIT TAPE
        otherActions.spitTape();
        sleep(750);
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
