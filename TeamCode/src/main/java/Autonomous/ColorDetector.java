package Autonomous;

import android.graphics.Bitmap;

/**
 * Author: Ethan Fisher
 * Date: 10/26/2020
 *
 * Detects rings easily
 */

public class ColorDetector {

    private VuforiaHelper vuforia;
    public static final int TARGET_WIDTH = 100;
    public static final int TARGET_HEIGHT = 100;

    private enum NumRings { ZERO, ONE, FOUR }

    public static final int PERCENT_NUM_RINGS_REQUIRED = 70;
    public static final int TRIES_TO_GET_NUM_RINGS = 10;

    public static final int RED_PIXELS_REQUIRED = 100;

    private int targetR;
    private int targetG;
    private int targetB;
    private int tolerance;

    public ColorDetector(VuforiaHelper vuforia, int targetR, int targetG, int targetB, int tolerance) {
        this.vuforia = vuforia;
        this.targetR = targetR;
        this.targetG = targetG;
        this.targetB = targetB;
        this.tolerance = tolerance;
    }

    // used for getting rings
    public int getNumRings() {
        NumRings numRings = getNumRingsFound();

        if (numRings == NumRings.FOUR) return 4;
        if (numRings == NumRings.ONE) return 1;
        return 0;
    }

    // TODO use this function to find the amount of red in the screen. Test how much red
    // is a good amount to grab the wobble goal
    public boolean shouldGrabWobbleGoal() { return findNumDesiredPixels() > RED_PIXELS_REQUIRED; }

    private NumRings getNumRingsFound() {

        // number of rings
        int numZero = 0;
        int numOne = 0;
        int numFour = 0;

        // read 10 times or something like that
        for (int i = 0; i < TRIES_TO_GET_NUM_RINGS; i++) {
            int orangePixels = findNumDesiredPixels();

            if (orangePixels < 25)
                numZero++;
            else if (orangePixels < 250)
                numOne++;
            else
                numFour++;
        }

        // check if the percent of rings found is enough to assume that number of rings are on the field
        if (numZero / (double)TRIES_TO_GET_NUM_RINGS >= PERCENT_NUM_RINGS_REQUIRED)
            return NumRings.ZERO;
        else if (numOne / (double)TRIES_TO_GET_NUM_RINGS >= PERCENT_NUM_RINGS_REQUIRED)
            return NumRings.ONE;
        else
            return NumRings.FOUR;
    }

    public int findNumDesiredPixels() {

        Bitmap image = vuforia.getImage(TARGET_WIDTH, TARGET_HEIGHT);
        if (image == null) return 0;

        // RGB for orange = #FFa500
        // variables for the bounds of the orange allowed for each pixel
        int redMin = targetR - tolerance;
        int redMax = targetR + tolerance;
        int greenMin = targetG - tolerance;
        int greenMax = targetG + tolerance;
        int blueMin = targetB - tolerance;
        int blueMax = targetB + tolerance;

        int orangePixels = 0;

        for (int i = 0; i < image.getWidth(); i++)
            for (int j = 0; j < image.getHeight(); j++) {
                int pixel = image.getPixel(i, j);

                // convert pixel to a color
//                int A = (pixel >> 24) & 0xff; // or color >>> 24
                int R = (pixel >> 16) & 0xff;
                int G = (pixel >>  8) & 0xff;
                int B = (pixel      ) & 0xff;

                if (R >= redMin && R <= redMax && G >= greenMin && G <= greenMax && B >= blueMin && B <= blueMax)
                    orangePixels++;
            }
        return orangePixels;
    }

}
