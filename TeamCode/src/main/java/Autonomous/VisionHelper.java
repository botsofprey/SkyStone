package Autonomous;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone.LABEL_STONE;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone.LABEL_SKY_STONE;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone.TFOD_MODEL_ASSET;

/**
 * Created by robotics on 12/18/18.
 */

// TODO: update for this season
public class VisionHelper extends Thread {
//    public final static int LEFT = 0, CENTER = 1, RIGHT = 2, NOT_DETECTED = -1;
    public final static int SLEEP_TIME_MILLIS = 200;
    public final static int PHONE_CAMERA = 0;
    public final static int WEBCAM = 1;
    public final static int LOCATION = 0/*, MINERAL_DETECTION = 1*/, STONE_DETECTION = 1, BOTH = 2;
    private final int POSITION_VOTE_MINIMUM_COUNT = 15;
    VuforiaLocalizer vuforia;
    VuforiaTrackables targetsRoverRuckus;
    VuforiaTrackables targetsSkyStone;
    VuforiaTrackable stoneTarget;
    VuforiaTrackable blueRearBridge;
    VuforiaTrackable redRearBridge;
    VuforiaTrackable redFrontBridge;
    VuforiaTrackable blueFrontBridge;
    VuforiaTrackable red1;
    VuforiaTrackable red2;
    VuforiaTrackable front1;
    VuforiaTrackable front2;
    VuforiaTrackable blue1;
    VuforiaTrackable blue2;
    VuforiaTrackable rear1;
    VuforiaTrackable rear2;
    private volatile TFObjectDetector tfod;
//    private volatile double[] positionVotes = {0, 0, 0};
    private volatile boolean running = true/*, detectingGold = false*/, trackingLocation = false;
    private volatile boolean findingSkyStone = false;
    private volatile boolean targetVisible = false;
    private volatile OpenGLMatrix lastLocation = null;
    private volatile Location robotLocation = new Location(0, 0);
    List<VuforiaTrackable> allTrackables;
    Orientation robotOrientation;
    VectorF translation;
    private int mode = LOCATION;

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;  // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (5.75f) * mmPerInch;    // the height of the center of the target image above the floor

    final int CAMERA_FORWARD_DISPLACEMENT_FROM_CENTER = (int)(1.5*mmPerInch);
    final int CAMERA_VERTICAL_DISPLACEMENT_FROM_CENTER = (int)(14*mmPerInch);
    final int CAMERA_LEFT_DISPLACEMENT_FROM_CENTER = (int)(-mmPerInch);

    public VisionHelper(int camera, HardwareMap hardwareMap) {
        initBoth(camera, hardwareMap);
    }

    public VisionHelper(int camera, int mode, HardwareMap hardwareMap) {
        switch (mode) {
            case LOCATION:
                mode = LOCATION;
                vuforia = VuforiaHelper.initVuforia(camera, hardwareMap);
                break;
//            case MINERAL_DETECTION:
//                mode = MINERAL_DETECTION;
//                initBoth(camera, hardwareMap);
//                break;
            case STONE_DETECTION:
                mode = STONE_DETECTION;
                initBoth(camera, hardwareMap);
                break;
            case BOTH:
                mode = BOTH;
                initBoth(camera, hardwareMap);
                break;
            default:
                break;
        }
    }

    private void initBoth(int camera, HardwareMap hardwareMap) {
        try {
            vuforia = VuforiaHelper.initVuforia(camera, hardwareMap);
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_SKY_STONE, LABEL_STONE);
//            tfod.activate();
        } catch (Exception e) {
            Log.e("VisionHelper Error", e.toString());
            throw new RuntimeException(e);
        }
    }

    @Override
    public void run() {
        if(tfod != null) {
            while (running) {
//                if(detectingGold) updatePositionVotesRightTwoVisible();
//                if(trackingLocation) updateRobotLocation();
               if (findingSkyStone) getStonesInView();
                try {
                    sleep(SLEEP_TIME_MILLIS);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
//            resetPositionVotes();
        }
    }

    public void startDetection() {
        loadNavigationAssets();
//        resetPositionVotes(); // TODO: update
        robotOrientation = new Orientation(EXTRINSIC, XYZ, DEGREES, 0, 0, 0, 0);
        running = true;
        findingSkyStone = true;
        this.start();
    }

    public void stopDetection() { // TODO: update
//        detectingGold = false;
//        trackingLocation = false;
        findingSkyStone = false;
        running = false;
    }

//    public void startGoldDetection() {
//        detectingGold = true;
//    }

    public void startSkyStoneDetection() { findingSkyStone = true; }

//    public void stopGoldDetection() {
//        detectingGold = false;
//    }

    public void startTrackingLocation() {
        trackingLocation = true;
    }

    public void stopTrackingLocation() {
        trackingLocation = false;
    }

    public Recognition[] getStonesInView() {
        Recognition[] recognitions = new Recognition[50];
        List<Recognition> Recognitions = tfod.getRecognitions();

        for (int i = 0; i < tfod.getRecognitions().size(); i++) recognitions[i] = Recognitions.get(i);

        if (recognitions != null) {
            return filterStonesOnScreen(recognitions);
        }
        return null;
    }

//    public Recognition getClosestMineral() {
//        List<Recognition> recognitions = tfod.getRecognitions();
//        if(recognitions != null) {
//            Recognition[] minerals = filterMineralsOnScreen(recognitions);
//            return minerals[0];
//        }
//        return null;
//    }

//    public int getGoldMineralPosition() {
//        int position = NOT_DETECTED;
//        if(positionVotes[LEFT] + positionVotes[CENTER] + positionVotes[RIGHT] >= POSITION_VOTE_MINIMUM_COUNT) {
//            if(positionVotes[LEFT] > positionVotes[CENTER] && positionVotes[LEFT] > positionVotes[RIGHT]) {
//                position = LEFT;
//            } else if(positionVotes[RIGHT] > positionVotes[CENTER] && positionVotes[RIGHT] > positionVotes[LEFT]) {
//                 position = RIGHT;
//            } else {
//                position = CENTER;
//            }
//        }
//        return position;
//    }

//    public int getLargestPositionVote() {
//        if(positionVotes[LEFT] > 0 || positionVotes[RIGHT] > 0 || positionVotes[CENTER] > 0) {
//            if (positionVotes[LEFT] > positionVotes[RIGHT]) {
//                if (positionVotes[LEFT] > positionVotes[CENTER]) return LEFT;
//                else return CENTER;
//            } else if (positionVotes[RIGHT] > positionVotes[CENTER]) return RIGHT;
//            else return CENTER;
//        } else return NOT_DETECTED;
//    }

//    public void addPositionVote(int position) {
//        positionVotes[position]++;
//    }

    public Orientation getRobotOrientation() {
        return robotOrientation;
    }

    public double getRobotHeading() {
//        double heading = -robotOrientation.thirdAngle;
//        return heading;
        return -robotOrientation.thirdAngle;
    }

    public Location getRobotLocation() {
        if(trackingLocation) return robotLocation;
        else return null;
    }

//    public void resetPositionVotes() {
//        for(int i = 0; i < positionVotes.length; i++) {
//            positionVotes[i] = 0;
//        }
//    }

//    public double getPositionVote(int mineralLocation) {
//        return positionVotes[mineralLocation];
//    }

//    private void updatePositionVotes() {
//        List<Recognition> recognitions = tfod.getRecognitions();
//        if(recognitions != null) {
//            Recognition[] minerals = filterMineralsOnScreen(recognitions);
//            if(minerals.length >= 3) {
//                int goldMineralX = -1;
//                int silverMineral1X = -1;
//                int silverMineral2X = -1;
//                for (Recognition recognition : minerals) {
//                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
//                        goldMineralX = (int) recognition.getLeft();
//                    } else if (silverMineral1X == -1) {
//                        silverMineral1X = (int) recognition.getLeft();
//                    } else {
//                        silverMineral2X = (int) recognition.getLeft();
//                    }
//                }
//                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
//                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
//                        positionVotes[LEFT]++;
//                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                        positionVotes[RIGHT]++;
//                    } else {
//                        positionVotes[CENTER]++;
//                    }
//                }
//            }
//        }
//    }

//    private void updatePositionVotesRightTwoVisible() {
//        List<Recognition> recognitions = tfod.getRecognitions();
//        if(recognitions != null) {
//            Recognition[] temp = filterMineralsOnScreen(recognitions);
//            Recognition[] minerals = null;
//            if(temp != null && temp.length >= 2) {
//                minerals = new Recognition[] {temp[0], temp[1]};
//            }
//            if(minerals != null && minerals.length >= 2) {
//                int goldMineralX = -1;
//                int silverMineralX = -1;
//                for (Recognition recognition : minerals) {
//                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
//                        goldMineralX = (int) recognition.getLeft();
//                    } else {
//                        silverMineralX = (int) recognition.getLeft();
//                    }
//                }
//                if (silverMineralX != -1) {
//                    if(goldMineralX != -1) {
//                        if (goldMineralX < silverMineralX) {
//                            positionVotes[CENTER]++;
//                        } else if (goldMineralX > silverMineralX) {
//                            positionVotes[RIGHT]++;
//                        }
//                    } else {
//                        positionVotes[LEFT]++;
//                    }
//                }
//            }
//        }
//    }

    private void updateRobotLocation() {
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        if (targetVisible) {
            translation = lastLocation.getTranslation();
            robotLocation = new Location(0, 0);
            robotLocation.updateXY(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch);
            robotOrientation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
        }
        else {
            robotLocation = null;
            robotOrientation = null;
        }
    }

//    private Recognition[] filterMineralsOnScreen(List<Recognition> stones) {
//        Recognition[] stonesArray = stones.toArray(new Recognition[0]);
//        Arrays.sort(stonesArray, new Comparator<Recognition>() {
//            @Override
//            public int compare(Recognition r1, Recognition r2) {
//                return (int) (r2.getBottom() - r1.getBottom());
//            }
//        });
//        return stonesArray;
//    }

    private Recognition[] filterStonesOnScreen(Recognition[] stones) {
        Arrays.sort(stones, new Comparator<Recognition>() {
            @Override
            public int compare(Recognition r1, Recognition r2) {
                return (int) (r2.getLeft() - r1.getLeft());
            }
        });
        return stones;
    }

    public void loadNavigationAssets() {
        targetsSkyStone = vuforia.loadTrackablesFromAsset("SkyStone");
        stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        allTrackables = new ArrayList<>();
        allTrackables.addAll(targetsSkyStone);
//
//        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
//                .translation(2*mmFTCFieldWidth, mmFTCFieldWidth, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 270));
//        blueRover.setLocation(blueRoverLocationOnField);
//
//        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
//                .translation(2*mmFTCFieldWidth, mmFTCFieldWidth, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 270));
//        redFootprint.setLocation(redFootprintLocationOnField);
//
//        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
//                .translation(mmFTCFieldWidth, 2*mmFTCFieldWidth, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 180));
//        frontCraters.setLocation(frontCratersLocationOnField);
//
//        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
//                .translation(mmFTCFieldWidth, 2*mmFTCFieldWidth, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
//        backSpace.setLocation(backSpaceLocationOnField);
//
//        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
//                .translation(CAMERA_FORWARD_DISPLACEMENT_FROM_CENTER, CAMERA_LEFT_DISPLACEMENT_FROM_CENTER, CAMERA_VERTICAL_DISPLACEMENT_FROM_CENTER)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
//                        90, 0, 90));
//
//        for (VuforiaTrackable trackable : allTrackables)
//        {
//            ((VuforiaTrackableDefaultListener)trackable.getListener()).setCameraLocationOnRobot(vuforia.getCameraName(), cameraLocationOnRobot);
//        }

//        targetsSkyStone.activate();
        tfod.activate();
    }

    public void kill() {
        stopDetection();
        if(mode == BOTH || mode == STONE_DETECTION) tfod.shutdown();
//        Vuforia.deinit();
    }
}
