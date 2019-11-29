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

import Actions.StoneStackingSystem;
import Autonomous.Location;
import Autonomous.VisionHelper;
import DriveEngine.JennyNavigation;
import SensorHandlers.Sensor;
import SensorHandlers.SensorPackage;

@Autonomous(name="Sensor Package Test", group="Linear Opmode")
//@Disabled
public class SensorPackageTest extends LinearOpMode {
    // create objects and locally global variables here
    JennyNavigation robot;
    StoneStackingSystem sss;
    VisionHelper vision;
    SensorPackage sensors;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        sensors = new SensorPackage(new Sensor(hardwareMap.get(DistanceSensor.class, "left"), 0), new Sensor(hardwareMap.get(DistanceSensor.class, "back"), 1));
        // initialize objects and variables here
        // also create and initialize function local variables here
        try {
            robot = new JennyNavigation(hardwareMap, new Location(0, 0), 0, "RobotConfig/AnnieV1.json");
        } catch (Exception e) {
            e.printStackTrace();
        }
        sss = new StoneStackingSystem(hardwareMap);

        // Stone detection
        vision = new VisionHelper(VisionHelper.WEBCAM, VisionHelper.LOCATION, hardwareMap);
        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // nothing goes between the above and below lines
        waitForStart();

//        vision.startDetection();
//        vision.startTrackingLocation();
//
////        LOOP 3 TIMES:
////        for(int i = 0; i < 3; i++) {
////          Get distance from wall
////          Move right following wall
////          Look for skystone
//
//        final double DISTANCE_TOLERANCE = .5; //IN INCHES
//        final double ANGLE_TOLERANCE = 1;
//        boolean ready = false;
//
//        if(opModeIsActive()) robot.driveDistance(15, 0, 10, this);
//        while (opModeIsActive() && !ready) {
//            if (vision.getSkystoneOrientation() != null) {
//                telemetry.addData("Skystone Location", vision.getSkystoneLocation() + ", " + vision.getSkystoneOrientation().thirdAngle);
//                robot.orbitSkystone(vision.getSkystoneLocation(), vision.getSkystoneOrientation().thirdAngle, 10, this);
//
//                if (Math.abs(vision.getSkystoneLocation().getY()) < DISTANCE_TOLERANCE && Math.abs(vision.getSkystoneLocation().getX()) < 11 && Math.abs(vision.getSkystoneOrientation().thirdAngle) < ANGLE_TOLERANCE) {
//                    ready = true;
//                    robot.brake();
//                }
//            } else {
//                robot.brake();
//            }
//            telemetry.update();
//        }
//        telemetry.addData("Status", "Reached the skystone. Time to pick it up!");
//        if(opModeIsActive()) robot.driveDistance(7, 0, 10, this);
//        if(opModeIsActive()) sss.grabStoneCenter();
//        sleep(500);
//        if(opModeIsActive()) robot.driveDistance(10, 180, 10, this);
//        if(opModeIsActive()) robot.driveDistance(72, 90,25,this);
//        if(opModeIsActive()) sss.releaseStoneCenter();
//                Finding any stones using TensorFlow
//                Recognition[] recognitions = vision.getStonesInView();
//                if (recognitions != null) {
////                    for (Recognition r : recognitions) {
////                        if(r.getLabel() == LABEL_STONE) {
////                            telemetry.addData("Skystone found", r.getLeft());
////                        }
////                    }
//                }
        while (opModeIsActive()) {
            telemetry.addData("Left Sensor", sensors.getSensor(DistanceSensor.class,0));
            telemetry.addData("Left Dist", sensors.getSensor(DistanceSensor.class, 0).getDistance(DistanceUnit.INCH));
            telemetry.addData("Back Sensor", sensors.getSensor(DistanceSensor.class,1));
            telemetry.addData("Back Dist", sensors.getSensor(DistanceSensor.class, 1).getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
//          IF i < 2 skystone found
//              Move Forward
//              Grab with central gripper
//              Move Backward
//          ELSE
//              Go to right wall
//              Move Forward
//              Grab first block found
//              Move Backward
//          Move left to foundation
//          Place block
//        }


//        robot.driveDistance(24,180, 35, this);

////        THIS IS FOR GRABBING FOUNDATION
//        robot.driveDistance(10, 270, 15, this);
//        robot.driveDistance(30, 270, 15, this);
//        if(opModeIsActive())sss.extendRightArm();
//        sleep();
//        sss.pauseRightArm();
//        if(opModeIsActive())sss.extendLeftArm();
//        if(opModeIsActive())sss.setLeftArmPosition(180);
//        if(opModeIsActive())sss.setRightArmPosition(180);
//        robot.driveDistance(38.42, 336, 15, this);
//
////         For future implementation: get stones
//        int reps = 6;
//        for (int i = 0; i < reps && opModeIsActive(); i++){
//            robot.driveDistance(20,0,25,this);
//            robot.driveDistance(87 + i * 8,90,25,this);
//            robot.driveDistance(24,180,25,this);
//            robot.driveDistance(89 + i * 8, 270, 25,this);
//        }

//
        vision.kill();
        robot.stopNavigation();
        sss.kill();

        // finish drive code and test
        // may be a good idea to square self against wall

    }
    // misc functions here
}
