/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package Autonomous.OpModes.UltimateAuto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Autonomous.AutoAlliance;
import DriveEngine.Ultimate.UltimateNavigation;

import static Autonomous.ConfigVariables.RING_CHECKPOINT;

/*
    Author: Ethan Fisher
    Date: 10/29/2020

    An opmode for the Ultimate Goal Autonomous
 */
@Autonomous(name="UltimateV1Auto", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class UltimateV1AutoRed extends LinearOpMode {

    @Override
    public void runOpMode() {

        // initialize robot
        final UltimateAutonomous robot = new UltimateAutonomous(AutoAlliance.RED, this);

        telemetry.addData("Robot created","");
        telemetry.update();

        // get number of rings and log them
//        int numRings = robot.getRingDetector().getNumRingsFound();
//        int numRings = robot.distanceRingsDetected();
//        telemetry.addData("Rings Found", numRings);
//        int numRings = robot.getRingDetector().getNumRingsFound();
        telemetry.addData("Status", "Initialized");
//        telemetry.addData("Rings Detected:", numRings);
        telemetry.update();

        // if (shouldNotPark(startTime)) goes before all of statements in case the 30 seconds is up

        // Wait for the game to start (driver presses PLAY)
//        while(!opModeIsActive()) {
//            numRings = robot.distanceRingsDetected();
//            telemetry.addData("Rings Found", numRings);
//            telemetry.update();
//        }
        waitForStart();

        long startTime = System.currentTimeMillis();

        // move to the zone with the wobble goal and release it
        if (shouldNotPark(startTime)) robot.getShooter().raiseElevator();
        if (shouldNotPark(startTime)) robot.getWobbleGrabber().grabWobbleGoal();
        if (shouldNotPark(startTime)) robot.driveToRingCheckpoint();
        sleep(1000);
        int numRings = robot.distanceRingsDetected();
        telemetry.addData("Rings Detected:", numRings);
        telemetry.update();
        if (shouldNotPark(startTime)) robot.driveToLocation(RING_CHECKPOINT);
        if (shouldNotPark(startTime)) robot.moveToZone(numRings);
        if (shouldNotPark(startTime)) robot.dropWobbleGoal();
        // if (shouldNotPark(startTime)) robot.getWobbleGrabber().raiseArm();

        // grab the second wobble goal
//        if (shouldNotPark(startTime)) robot.driveToLeftWobbleGoal();

        // move it to the same zone and drop it
//        if (shouldNotPark(startTime)) robot.driveToWaypoint();
//        if (shouldNotPark(startTime)) robot.moveToZone(numRings);
//        if (shouldNotPark(startTime)) robot.dropWobbleGoal();

        // move behind shot line, rotate towards powershots,  and shoot them
        if (shouldNotPark(startTime)) robot.moveToShootLine(); // TODO still need to work on calculations for shooting into goals & powershots
        //if (shouldNotPark(startTime)) robot.moveBehindShootLine();
        if (shouldNotPark(startTime)) robot.turnToZero();
        if (shouldNotPark(startTime)) robot.shootThreeRings();

        // drive back to the starting rings
//        if (shouldNotPark(startTime)) robot.driveToStartingRings();
//
//        // ??? Maybe grab three rings at the end ???
//        if (shouldNotPark(startTime)) robot.grabStartingPileRings();

        // park on the line and stop
        robot.park();
        robot.stop();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive());
    }
    
    public boolean shouldNotPark(long startTime) {
//        long curTimeMillis = System.currentTimeMillis() - startTime;
//        double curTimeSeconds = curTimeMillis / 1000.0;
//        return curTimeSeconds < 27;
        return true;
    }
}

