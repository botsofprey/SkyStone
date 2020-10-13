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
package Autonomous.OpModes;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.util.Scanner;

import UserControlled.RecordJSONTest;

/**
    Author: Ethan Fisher
    Date: 10/12/2020

    Replays a JSON file in the JSONFiles folder
 */
@Autonomous(name="Replay JSON Test", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class PlayJSONTest extends LinearOpMode {

    // filename for the current file being replayed
    public static final String PATH = "jsonFile.json";

    double[] leftPowers, rightPowers;

    DcMotor left, right;

    @Override
    public void runOpMode() {

        try {
            Scanner input = new Scanner(new File("TeamCode/src/main/java/Autonomous/OpModes/JSONFiles/" + PATH));
            JSONObject object = new JSONObject(input.nextLine());

            leftPowers = (double[]) object.get("left");
            rightPowers = (double[]) object.get("right");

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            final int TIMES_PER_SECOND = RecordJSONTest.TIMES_PER_SECOND;
            // record data 10 times a second (to start)
            long targetTime = 1000000000 / TIMES_PER_SECOND; // this value should be used in nano's
            long elapsedTime;
            long startTime;


            for (int i = 0; i < leftPowers.length; i++) {
                startTime = System.nanoTime();

                double leftPower = leftPowers[i];
                double rightPower = rightPowers[i];

                left.setPower(leftPower);
                right.setPower(rightPower);

                // convert time to milliseconds and sleep for the leftover time
                elapsedTime = startTime - System.nanoTime();
                sleep((targetTime - elapsedTime) / 1000000);

            }

        } catch(Exception e) {
            Log.d("Error", e.toString());
        }
    }

}