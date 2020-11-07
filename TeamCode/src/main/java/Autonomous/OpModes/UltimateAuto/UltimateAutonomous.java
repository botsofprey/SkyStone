package Autonomous.OpModes.UltimateAuto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Actions.Ultimate.RingIntakeSystemV1;
import Actions.Ultimate.ShooterSystemV1;
import Actions.Ultimate.WobbleGrabberV1;
import Autonomous.AutoAlliance;
import Autonomous.Location;
import Autonomous.RingDetector;
import Autonomous.VuforiaHelper;
import DriveEngine.Ultimate.UltimateNavigation;
import Autonomous.ConfigVariables;

import static Autonomous.ConfigVariables.RED_ZONE_ONE;
import static Autonomous.ConfigVariables.RED_ZONE_THREE;
import static Autonomous.ConfigVariables.RED_ZONE_TWO;
import static Autonomous.ConfigVariables.SHOOT_LINE;

/**
 * Author: Ethan Fisher
 * Date: 10/29/2020
 *
 * Autonomous for Ultimate Goal
 */
public class UltimateAutonomous {

    private final AutoAlliance alliance;
    private final LinearOpMode mode;
    private final HardwareMap hardwareMap;
    private static double MAX_SPEED = 25.0;  // inches / second

    private UltimateNavigation robot;
    private VuforiaHelper vuforia;
    private RingDetector ringDetector;

    private WobbleGrabberV1 wobbleGrabber;
    private ShooterSystemV1 shooter;
    private RingIntakeSystemV1 intake;

    public UltimateAutonomous(AutoAlliance alliance, LinearOpMode mode) {
        this.alliance = alliance;
        this.mode = mode;
        this.hardwareMap = mode.hardwareMap;

        initHardwareInterfaces();
        initNavigationSystem();
    }

    private void initHardwareInterfaces() {
        vuforia = new VuforiaHelper(hardwareMap);
        ringDetector = new RingDetector(vuforia);

        wobbleGrabber = new WobbleGrabberV1(hardwareMap);
        shooter = new ShooterSystemV1(hardwareMap);
        intake = new RingIntakeSystemV1(hardwareMap);
    }

    private void initNavigationSystem() {
        try {
            Location startLocation = redToBlue(new Location(62, -26.5, 270.0));
            robot = new UltimateNavigation(hardwareMap, startLocation, "RobotConfig/AnnieV1.json");
            robot.stopLoggingData();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    // converts red to blue. If it is blue, nothing happens
    public Location redToBlue(Location location) {
        if (alliance == AutoAlliance.BLUE) {
            double reflectedHeading = redToBlueHeading(location.getHeading());
            return new Location(-location.getX(), location.getY(), reflectedHeading);
        } else {
            return location;
        }
    }

    public double redToBlueHeading(double heading) {
        if (alliance == AutoAlliance.BLUE) {
            return 360.0 - heading;  // reflect heading angle
        } else {
            return heading;
        }
    }

    public void turnToZero() {
        robot.turnToHeading(0, mode);
    }

    public void driveToLocation(Location location) {
        robot.driveToLocationPID(redToBlue(location), MAX_SPEED, mode);
    }

    public void stop() { robot.stopNavigation(); }

    public void sleep(long milliseconds) { mode.sleep(milliseconds); }

    public void park() {
        // TODO spit tape after turning robot
        driveToLocation(SHOOT_LINE);
        sleep(800);
    }

    public void driveToWobbleGoal() {
        // TODO
    }

    public void moveToZone(int numRings) {
        // TODO (Check location measurements, assumed to be in cm)
        if(numRings == 0) {
            driveToLocation(RED_ZONE_ONE);
        }
        else if(numRings == 1) {
            driveToLocation(RED_ZONE_TWO);
        }
        else if(numRings == 4) {
            driveToLocation(RED_ZONE_THREE);
        }
    }

    public void moveBehindShootLine() {
        // TODO
    }

    public void shootPowerShots() {
        // TODO
    }

    public void grabStartingPileRings() {
        // TODO
    }

    public RingDetector getRingDetector() { return ringDetector; }
    public WobbleGrabberV1 getWobbleGrabber() { return wobbleGrabber; }
    public ShooterSystemV1 getShooter() { return shooter; }
    public RingIntakeSystemV1 getIntake() { return intake; }
}
