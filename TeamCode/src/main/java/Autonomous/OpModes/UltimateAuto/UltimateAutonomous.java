package Autonomous.OpModes.UltimateAuto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Actions.Ultimate.RingIntakeSystemV1;
import Actions.Ultimate.ShooterSystemV1;
import Actions.Ultimate.WobbleGrabberV1;
import Autonomous.AutoAlliance;
import Autonomous.Location;
import Autonomous.ColorDetector;
import Autonomous.VuforiaHelper;
import DriveEngine.Ultimate.UltimateNavigation;

import static Autonomous.ConfigVariables.PARKING_LOCATION;
import static Autonomous.ConfigVariables.RED_WOBBLE_GOAL_LEFT;
import static Autonomous.ConfigVariables.RED_WOBBLE_GOAL_LEFT_CHECKPOINT;
import static Autonomous.ConfigVariables.RED_ZONE_ONE;
import static Autonomous.ConfigVariables.RED_ZONE_THREE;
import static Autonomous.ConfigVariables.RED_ZONE_TWO;
import static Autonomous.ConfigVariables.SHOOTING_LINE_POINT;
//import static Autonomous.ConfigVariables.SHOOT_LINE;
import static Autonomous.ConfigVariables.STARTING_RING_PILE;
import static Autonomous.ConfigVariables.STARTING_ROBOT_LOCATION_RIGHT;
import static Autonomous.ConfigVariables.ZONE_WAYPOINT;

/**
 * Author: Ethan Fisher
 * Date: 10/29/2020
 *
 * Autonomous for Ultimate Goal
 */
public class UltimateAutonomous {

    // TODO test this class

    private final AutoAlliance alliance;
    private final LinearOpMode mode;

    public UltimateNavigation robot;
    private ColorDetector ringDetector;

    private WobbleGrabberV1 wobbleGrabber;
    private ShooterSystemV1 shooter;
    private RingIntakeSystemV1 intake;

    private static final double MAX_SPEED = UltimateNavigation.MAX_SPEED;

    public UltimateAutonomous(AutoAlliance alliance, LinearOpMode mode) {

        this.alliance = alliance;
        this.mode = mode;

        VuforiaHelper vuforia = new VuforiaHelper(mode.hardwareMap);
        ringDetector = new ColorDetector(vuforia, 0xFF, 0xa5, 0x00, 0x30);
        try {
            wobbleGrabber = new WobbleGrabberV1(mode.hardwareMap);
        }
        catch (Exception e) {
            e.printStackTrace();
        }

        shooter = new ShooterSystemV1(mode.hardwareMap);
        intake = new RingIntakeSystemV1(mode.hardwareMap);

        try {
            Location startLocation = redToBlue(STARTING_ROBOT_LOCATION_RIGHT);
            robot = new UltimateNavigation(mode.hardwareMap, startLocation, "RobotConfig/UltimateV1.json");
        } catch (Exception e) {
            mode.telemetry.addData("Robot error", e.toString());
        }
    }

    public void driveToLeftWobbleGoal() {
        driveToLocation(redToBlue(ZONE_WAYPOINT));
        driveToLocation(redToBlue(RED_WOBBLE_GOAL_LEFT_CHECKPOINT));

        wobbleGrabber.lowerArm();
        waitForArm();
        robot.driveDistance(8, UltimateNavigation.FORWARD, 5, mode);
//        robot.driveToLocationPID(RED_WOBBLE_GOAL_LEFT, 5, mode);

        // drive right a little bit maybe
        wobbleGrabber.grabWobbleGoal();
        sleep(600);
        wobbleGrabber.liftArm();
        waitForArm();
    }

//    public void driveToRightWobbleGoal() {
//        robot.turnToHeading(UltimateNavigation.WEST, mode);
//        Location target = new Location(RED_WOBBLE_GOAL_RIGHT);
//        target.setHeading(robot.orientation.getOrientation());
//        driveToLocation(target);
//    }

    public void driveToRightStartingPos() {
        Location target = new Location(STARTING_ROBOT_LOCATION_RIGHT);
        target.setHeading(robot.orientation.getOrientation());
        driveToLocation(target);
    }

    public void driveToWaypoint() {
        driveToLocation(redToBlue(ZONE_WAYPOINT));
    }

    public void moveToZone(int numRings) { // TODO needs a lot of testing
//        Location waypoint = new Location(ZONE_WAYPOINT);
//        waypoint.setHeading(robot.orientation.getOrientation());
//        driveToLocation(waypoint); // Travel to waypoint before moving to zone
        Location targetZone;

        if (numRings == 0)
            targetZone = new Location(RED_ZONE_ONE);
        else if (numRings == 1)
            targetZone = new Location(RED_ZONE_TWO);
        else
            targetZone = new Location(RED_ZONE_THREE);

//        double distanceY = targetZone.getY() - robot.getRobotLocation().getY();
//        robot.driveOnHeadingPID(UltimateNavigation.FORWARD, distanceY, mode);
//
//        double distanceX = targetZone.getX() - robot.getRobotLocation().getX();
//        robot.driveOnHeadingPID(UltimateNavigation.LEFT, distanceX, mode);

//        targetZone.setHeading(robot.orientation.getOrientation());
        driveToLocation(targetZone);
    }


      public void moveToShootLine() {
//        Location target = new Location(SHOOTING_LINE_POINT);
        driveToLocation(SHOOTING_LINE_POINT);
        sleep(500);
        robot.turnToHeading(UltimateNavigation.NORTH, mode);
        sleep(500);
      }

//    public void moveBehindShootLine() {
//        double zoneDistFromLine = (wobbleZone.getY() - SHOOT_LINE.getY()) / 2.54;
//        double distToDrive = zoneDistFromLine + 20;
//        robot.driveDistance(distToDrive, UltimateNavigation.SOUTH, MAX_SPEED, mode);
//        robot.brake();
//        sleep(1000);
//    }

    public void shootPowerShots() {
        shooter.raiseElevator();
        shooter.turnOnShooterWheel();
        sleep(1000);
        shooter.shoot();
        sleep(1000);
        shooter.turnOffShooterWheel();
    }

    public void driveToStartingRings() {
        // drive to the heading, within a certain range
        // TODO maybe move in the x and then in the y for more accuracy???
        robot.turnToHeading(UltimateNavigation.SOUTH, mode);
        double desiredDistanceFromRings = 2;
        robot.driveToLocationPID(STARTING_RING_PILE, MAX_SPEED, desiredDistanceFromRings, mode);
        sleep(1000);
    }

    public void grabStartingPileRings() {
        // turn towards the rings, then pick them up
        intake.turnOn();
        robot.driveDistance(10, MAX_SPEED, mode);
        robot.brake();
        sleep(1000);
        intake.turnOff();
    }

    public void park() {
//        driveToLocation(PARKING_LOCATION);
        robot.driveDistance(4, UltimateNavigation.FORWARD, 5, mode);
    }


    public void waitForArm() {
        while(mode.opModeIsActive() && wobbleGrabber.armIsBusy());
    }

    public void dropWobbleGoal() {
        robot.turnToHeading(135, mode);
//        wobbleGrabber.setArmAngle(45);
//        waitForArm();
//        sleep(200);
        wobbleGrabber.setArmAngle(80); // Slowly lowering the arm rather than slamming down goal
        waitForArm();
        sleep(200);
        wobbleGrabber.lowerArm();
        waitForArm();
        sleep(1000);
        wobbleGrabber.releaseWobbleGoal();
        sleep(1000);
        wobbleGrabber.setArmAngle(60);
        robot.turnToHeading(180, mode);
    }

    public void pickupWobbleGoal() {
        wobbleGrabber.lowerArm();
        waitForArm();
        wobbleGrabber.grabWobbleGoal();
        sleep(600);
        wobbleGrabber.raiseArm();
        waitForArm();

    }

    public void shootThreeRings() {
        shooter.raiseElevator();
        while(mode.opModeIsActive() && shooter.elevatorPosition != ShooterSystemV1.TOP) {
            shooter.update(mode);
        }
        shooter.shoot(); // Initally retracts indexer
        shooter.setShooter(ShooterSystemV1.HIGHEST_POSITION);
        shooter.turnOnShooterWheel();
        for(int i = 0; i <= 3; i++){
            sleep(1000); // Wait for shooter wheel to spin up
            shooter.shoot(); // Index ring into shooter
            sleep(500); // Wait for index
            shooter.shoot(); // Retract indexer
            sleep(500); // Wait for retract
        }
        shooter.turnOffShooterWheel();
        shooter.setShooter(ShooterSystemV1.LOWERED_POSITION);
//        while(mode.opModeIsActive() && shooter.elevatorPosition != ShooterSystemV1.BOTTOM)
//            shooter.update(mode);
        // Once sensors are functional, lower elevator
    }

    public void stop() { robot.stopNavigation(); }

    public void sleep(long milliseconds) { mode.sleep(milliseconds); }

    // converts red to blue. If it is blue, nothing happens
    public Location redToBlue(Location location) {
        if (alliance == AutoAlliance.BLUE)
            return new Location(-location.getX(), location.getY(), 360 - location.getHeading());
        else
            return location;
    }

    public void turnToZero() { robot.turnToHeading(UltimateNavigation.NORTH, mode); }

    public void driveToLocation(Location location) { robot.driveToLocationPID(redToBlue(location), MAX_SPEED, mode); }
    public ColorDetector getRingDetector() { return ringDetector; }
    //public void driveDistance(double distanceInInches, double heading){ robot.driveDistance(distanceInInches, heading, MAX_SPEED, mode); }
    public WobbleGrabberV1 getWobbleGrabber() { return wobbleGrabber; }
    public ShooterSystemV1 getShooter() { return shooter; }
    public RingIntakeSystemV1 getIntake() { return intake; }
}
