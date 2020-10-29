package Autonomous;

import android.graphics.Bitmap;

/**
 * Author: Ethan Fisher
 * Date: 10/26
 *
 * Detects rings easily
 */

public class RingDetector {

    private VuforiaHelper vuforia;
    public static final int TARGET_WIDTH = 100;
    public static final int TARGET_HEIGHT = 100;

    private enum NumRings { ZERO, ONE, FOUR }

    public static final int PERCENT_NUM_RINGS_REQUIRED = 7;
    public static final int TRIES_TO_GET_NUM_RINGS = 10;

    public RingDetector(VuforiaHelper vuforia) { this.vuforia = vuforia; }

    public int getNumRings() {
        NumRings numRings = getNumRingsFound();

        if (numRings == NumRings.FOUR) return 4;
        if (numRings == NumRings.ONE) return 1;
        return 0;
    }


    private NumRings getNumRingsFound() {

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

        if (numZero >= PERCENT_NUM_RINGS_REQUIRED)
            return NumRings.ZERO;
        else if (numOne >= PERCENT_NUM_RINGS_REQUIRED)
            return NumRings.ONE;
        else
            return NumRings.FOUR;
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

        if (orangePixels < 25)
            return NumRings.ZERO;
        else if (orangePixels < 250)
            return NumRings.ONE;
        else
            return NumRings.FOUR;
    }

}
