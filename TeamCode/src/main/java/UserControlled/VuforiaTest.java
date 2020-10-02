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
package UserControlled;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;

import Autonomous.VuforiaHelper;

/*
    An opmode for the User Controlled portion of the game
 */
@TeleOp(name="Image Capture Test", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class VuforiaTest extends LinearOpMode {

    private DcMotor left, right;
    private JoystickHandler joystick;

    private VuforiaHelper vuforia;

    @Override
    public void runOpMode() {

        left = hardwareMap.get(DcMotor.class, "leftMotor");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right = hardwareMap.get(DcMotor.class, "rightMotor");
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        joystick = new JoystickHandler(gamepad1, JoystickHandler.LEFT_JOYSTICK);

        vuforia = new VuforiaHelper(hardwareMap);
        telemetry.addData("Vuforia Working", "Height: " + vuforia.getImage(100, 100).getHeight() + "px");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            updateMotors();

            int[] colors = getAverageImageColor();
            if (colors != null) {
                telemetry.addData("Image", "Red: " + colors[0]);
                telemetry.addData("Image", "Green: " + colors[1]);
                telemetry.addData("Image", "Blue: " + colors[2]);
            };

            telemetry.update();
        }
    }

    private int[] getAverageImageColor() {
        Bitmap image = vuforia.getImage(100, 100);
        if (image == null) return null;

//        ArrayList<Color> colors = new ArrayList<>();
        for (int i = 0; i < image.getWidth(); i++) {

            for (int j = 0; j < image.getHeight(); j++) {
                int pixel = image.getPixel(i, j);
                int A = (pixel >> 24) & 0xff; // or color >>> 24
                int R = (pixel >> 16) & 0xff;
                int G = (pixel >>  8) & 0xff;
                int B = (pixel      ) & 0xff;
                return new int[] { R, G, B };
            }
        }
        return new int[] { 1, 2, 3 };
    }

    private void updateMotors() {
        double leftPower, rightPower;
        leftPower = joystick.y() + joystick.x();
        rightPower = joystick.y() - joystick.x();

        left.setPower(leftPower);
        right.setPower(rightPower);

//        telemetry.addData("Drive Power Left", leftPower);
//        telemetry.addData("Drive Power Right", rightPower);
    }
}