package UserControlled;

import com.qualcomm.robotcore.hardware.Gamepad;

/*
    Author: Ethan Fisher
    Date: 10/23/2020

    Makes it easier to use the gamepads
    For example, if you want to only allow the user to press a button
    and not hold it, this object will allow you to do so
 */
public class GamepadController {

    // TODO finish this up

    private boolean aPressed, aHeld, bPressed, bHeld, xPressed, xHeld, yPressed, yHeld,
        rightTriggerPressed, rightTriggerHeld, leftTriggerPressed, leftTriggerHeld,
        dpadUpPressed, dpadUpHeld, dpadRightPressed, dpadRightHeld, dpadDownPressed,
        dpadDownHeld, dpadLeftPressed, dpadLeftHeld;

    private Gamepad gamepad;

    public GamepadController(Gamepad gamepad) {
        this.gamepad = gamepad;
        update();
    }

    public void update() {

        // update a
        if (gamepad.a)
            aHeld = true;
        if (gamepad.a && !aPressed)
            aPressed = true;
        else if (!gamepad.a) {
            aHeld = false;
            aPressed = false;
        }

        // update b

        // update x

        // update y
    }

    public boolean aPressed() { return aPressed; }
    public boolean aHeld() { return aHeld; }

}
