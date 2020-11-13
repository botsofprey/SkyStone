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

import static Autonomous.ConfigVariables.RED_WOBBLE_GOAL_LEFT;
import static Autonomous.ConfigVariables.RED_ZONE_ONE;
import static Autonomous.ConfigVariables.RED_ZONE_THREE;
import static Autonomous.ConfigVariables.RED_ZONE_TWO;
import static Autonomous.ConfigVariables.SHOOT_LINE;
import static Autonomous.ConfigVariables.STARTING_RING_PILE;
import static Autonomous.ConfigVariables.STARTING_ROBOT_LOCATION_LEFT;

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
    private final HardwareMap hardwareMap;

    private UltimateNavigation robot;
    private VuforiaHelper vuforia;
    private ColorDetector ringDetector;

    private WobbleGrabberV1 wobbleGrabber;
    private ShooterSystemV1 shooter;
    private RingIntakeSystemV1 intake;

    private static final double MAX_SPEED = UltimateNavigation.MAX_SPEED;

    private Location wobbleZone;

    public UltimateAutonomous(AutoAlliance alliance, LinearOpMode mode) {

        this.alliance = alliance;
        this.mode = mode;
        this.hardwareMap = mode.hardwareMap;

        vuforia = new VuforiaHelper(hardwareMap);
        ringDetector = new ColorDetector(vuforia, 0xFF, 0xa5, 0x00, 0x30);

        wobbleGrabber = new WobbleGrabberV1(hardwareMap);
        shooter = new ShooterSystemV1(hardwareMap);
        intake = new RingIntakeSystemV1(hardwareMap);

        try {
            Location startLocation = redToBlue(STARTING_ROBOT_LOCATION_LEFT);
            robot = new UltimateNavigation(hardwareMap, startLocation, "RobotConfig/UltimateV1.json");
        } catch (Exception e) {
            mode.telemetry.addData("Robot error", e.toString());
            mode.telemetry.update();
        }
    }

    public void driveToWobbleGoal() {
        robot.driveToLocation(RED_WOBBLE_GOAL_LEFT, MAX_SPEED, mode);
    }

    public void moveToZone(int numRings) {
        if (numRings == 0)
            driveToLocation(wobbleZone = RED_ZONE_ONE);

        else if (numRings == 1)
            driveToLocation(wobbleZone = RED_ZONE_TWO);

        else if (numRings == 4)
            driveToLocation(wobbleZone = RED_ZONE_THREE);
    }

    public void moveBehindShootLine() {
        double zoneDistFromLine = (wobbleZone.getY() - SHOOT_LINE.getY()) / 2.54;
        double distToDrive = zoneDistFromLine + 20;
        robot.driveDistance(distToDrive, UltimateNavigation.SOUTH, MAX_SPEED, mode);
        robot.brake();
        sleep(1000);
    }

    public void shootPowerShots() {
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
        robot.driveToLine(SHOOT_LINE, MAX_SPEED, mode);
        sleep(800);
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

    public void turnToZero() { robot.turnToHeading(0, mode); }

    public void driveToLocation(Location location) { robot.driveToLocationPID(redToBlue(location), MAX_SPEED, mode); }

    public ColorDetector getRingDetector() { return ringDetector; }
    public WobbleGrabberV1 getWobbleGrabber() { return wobbleGrabber; }
    public ShooterSystemV1 getShooter() { return shooter; }
    public RingIntakeSystemV1 getIntake() { return intake; }
}
