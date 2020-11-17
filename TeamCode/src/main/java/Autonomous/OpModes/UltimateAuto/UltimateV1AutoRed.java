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

import static Autonomous.ConfigVariables.CENTER;

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
        UltimateAutonomous robot = new UltimateAutonomous(AutoAlliance.RED, this);

        telemetry.addData("Robot Created", "");
        telemetry.update();


        // get number of rings and log them
      telemetry.addData("About to get ring detector...", "");
        int numRings = 4; //robot.getRingDetector().getNumRings();
        telemetry.addData("Rings Found", numRings);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // if (opModeIsActive()) goes before all of statements in case the 30 seconds is up

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // TODO the robot should start with the wobble goal
//        // drive to the wobble goal
//        robot.driveToWobbleGoal();
//
//        // grab the wobble goal
//        robot.getWobbleGrabber().grabWobbleGoal();

        // move to the zone with the wobble goal and release it
        if (opModeIsActive()) robot.getWobbleGrabber().grabWobbleGoal();
        if (opModeIsActive()) robot.moveToZone(numRings);
        if (opModeIsActive()) robot.robot.turnToHeading(90, this);
        if (opModeIsActive()) robot.placeWobbleGoal(); // take second look at, isn't doing anything at the moment. same for pickup method.

        // grab the second wobble goal
        if (opModeIsActive()) robot.driveToSecondWobbleGoal();
        if (opModeIsActive()) robot.robot.turnToHeading(-90,this);
        if (opModeIsActive()) robot.pickupWobbleGoal();

        // move it to the same zone and drop it
        if (opModeIsActive()) robot.moveToZone(numRings);
        if (opModeIsActive()) robot.placeWobbleGoal();

        // move behind shot line, rotate towards powershots,  and shoot them
        if (opModeIsActive()) robot.moveBehindShootLine();
        if (opModeIsActive()) robot.turnToZero();
        if (opModeIsActive()) robot.shootPowerShots();

        // drive back to the starting rings
        if (opModeIsActive()) robot.driveToStartingRings();

        // ??? Maybe grab three rings at the end ???
        if (opModeIsActive()) robot.grabStartingPileRings();

        // park on the line and stop
        if (opModeIsActive()) robot.park();
        robot.stop();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive());

    }
}

