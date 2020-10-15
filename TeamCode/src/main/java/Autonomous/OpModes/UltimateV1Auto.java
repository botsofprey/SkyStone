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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;

import Autonomous.VuforiaHelper;
import UserControlled.JoystickHandler;

/*
    An opmode for the User Controlled portion of the game
 */
@Autonomous(name="UltimateV1Auto", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class UltimateV1Auto extends LinearOpMode {

    private VuforiaHelper vuforia;
    public static final int TARGET_WIDTH = 100;
    public static final int TARGET_HEIGHT = 100;

    private enum NumRings { ZERO, ONE, FOUR }

    public static final int PERCENT_NUM_RINGS_REQUIRED = 7;
    public static final int TRIES_TO_GET_NUM_RINGS = 10;

    @Override
    public void runOpMode() {

        vuforia = new VuforiaHelper(hardwareMap);

        // default number of found rings to zero
        NumRings numRingsFound = getNumRingsFound();

        telemetry.addData("Rings Found", numRingsFound);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            findNumRings();

            telemetry.update();
        }
    }

    public NumRings getNumRingsFound() {

        // number of rings
        int numZero = 0;
        int numOne = 0;
        int numFour = 0;

        // read 10 times or something like that
        for (int i = 0; i < TRIES_TO_GET_NUM_RINGS; i++) {
            NumRings num = findNumRings();
            if (num == null) continue;

            if (num == NumRings.ZERO)
                numZero++;
            else if (num == NumRings.ONE)
                numOne++;
            else
                numFour++;
        }

        NumRings numRingsFound;
        if (numZero >= PERCENT_NUM_RINGS_REQUIRED)
            numRingsFound = NumRings.ZERO;
        else if (numOne >= PERCENT_NUM_RINGS_REQUIRED)
            numRingsFound = NumRings.ONE;
        else
            numRingsFound = NumRings.FOUR;
        return numRingsFound;
    }

    private NumRings findNumRings() {

        Bitmap image = vuforia.getImage(TARGET_WIDTH, TARGET_HEIGHT);
        if (image == null) return null;

        // RGB for orange = #FFa500
        // variables for the bounds of the orange allowed for each pixel
        int redMin = 0xAA;
        int greenMin = 0x55;
        int greenMax = 0xCC;
        int blueMax = 0x44;

        int orangePixels = 0;

        for (int i = 0; i < image.getWidth(); i++) {

            for (int j = 0; j < image.getHeight(); j++) {
                int pixel = image.getPixel(i, j);

                // convert pixel to a color
//                int A = (pixel >> 24) & 0xff; // or color >>> 24
                int R = (pixel >> 16) & 0xff;
                int G = (pixel >>  8) & 0xff;
                int B = (pixel      ) & 0xff;

                if (R >= redMin && G >= greenMin && G <= greenMax && B <= blueMax)
                    orangePixels++;
            }
        }

        telemetry.addData("Orange Pixels: ", orangePixels);

        if (orangePixels < 25)
            return NumRings.ZERO;
        else if (orangePixels < 250)
            return NumRings.ONE;
        else
            return NumRings.FOUR;
    }
}