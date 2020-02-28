package Autonomous;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ConfigVariables {
    public static double STONE_ONE_LEFT = 36.5;
    public static double STONE_ONE_RIGHT = 30.5;
    public static double STONE_TWO_LEFT = 26.5;
    public static double STONE_TWO_RIGHT = 23.5;
    public static double STONE_THREE_LEFT = 19;
    public static double STONE_THREE_RIGHT = 15;

    public static double DIST_STONE_ONE_LEFT = 38.5;
    public static double DIST_STONE_ONE_RIGHT = 35;
    public static double DIST_STONE_TWO_LEFT = 28.0;
    public static double DIST_STONE_TWO_RIGHT = 28.5;
    public static double DIST_STONE_THREE_LEFT = 23;
    public static double DIST_STONE_THREE_RIGHT = 20;

    public static double STONE_FOUR = 28.5;
    public static double STONE_FIVE = 27;
    public static double STONE_SIX = 19.5;

    public static double DIST_STONE_FOUR = 11.3;
    public static double DIST_STONE_FIVE = 5.3;
    public static double DIST_STONE_SIX = 5;

    public static Location UNDER_RED_BRIDGE = new Location(40, 0, 270);
    public static Location UNDER_RED_BRIDGE_0_HEADING = new Location(40, 0, 0);
    public static Location UNDER_RED_BRIDGE_BUILDING_ZONE = new Location(40, 10, 0);
    public static Location UNDER_RED_BRIDGE_LOADING_ZONE = new Location(40, -10, 0);
    public static Location BEHIND_RED_QUARRY = new Location(40, -38, 270);
    public static Location FIRST_STONE_GROUP_CENTER_RED = new Location(24+12, -58.5, 270);
    public static Location FIRST_STONE_GROUP_LEFT_RED = new Location(24+12, -66.5, 270);
    public static Location FIRST_STONE_GROUP_RIGHT_RED = new Location(24+12, -50.5, 270);
    public static Location SECOND_STONE_GROUP_CENTER_RED = new Location(24+12, -34.5, 270);
    public static Location SECOND_STONE_GROUP_RIGHT_RED = new Location(24+12, -26.5, 270);
    public static Location SECOND_STONE_GROUP_LEFT_RED = new Location(24+12, -42.5, 270); // should be 34.25, but robot stops too soon
    public static Location RED_CALIBRATE_ZONE_1 = new Location(35, 35, 0);
    public static Location RED_CALIBRATE_ZONE_2 = new Location(35, -35, 0);

    public static Location FIRST_STONE_GROUP_CENTER_BLUE = new Location(-24-9, -38, 90);
    public static Location FIRST_STONE_GROUP_LEFT_BLUE = new Location(-24-9, -32, 90);
    public static Location FIRST_STONE_GROUP_RIGHT_BLUE = new Location(-24-9, -44, 90);
    public static Location SECOND_STONE_GROUP_CENTER_BLUE = new Location(-24-9, -56, 90);
    public static Location SECOND_STONE_GROUP_LEFT_BLUE = new Location(-24-9, -50, 90);
    public static Location SECOND_STONE_GROUP_RIGHT_BLUE = new Location(-24-9, -62, 90);

    public static Location RED_FOUNDATION_CENTER = new Location(24+11+6.5, 49.25, 270);
    public static Location RED_FOUNDATION_STACK_CENTER = new Location(32, 49.25, 270);
    public static Location RED_FOUNDATION_LEFT = new Location(39.5, 37.5, 270);
    public static Location RED_FOUNDATION_STACK_LEFT = new Location(20+9+4, 37.5, 270);
    public static Location BLUE_FOUNDATION_CENTER = new Location(-24-10-5, 49.5, 90);

    public static Rectangle VALID_Y_SENSOR_READ_AREA_1_RED = new Rectangle(34, 48, 12, 48);
    public static Rectangle VALID_Y_SENSOR_READ_AREA_2_RED = new Rectangle(34, -48, 12, 48);
    public static Rectangle VALID_X_SENSOR_READ_AREA_1_RED = new Rectangle(48, 34, 48, 12);
    public static Rectangle VALID_X_SENSOR_READ_AREA_2_RED = new Rectangle(48, -34, 48, 12);

    public static Rectangle VALID_Y_SENSOR_READ_AREA_1_BLUE = new Rectangle(-34, 48, 12, 48);
    public static Rectangle VALID_Y_SENSOR_READ_AREA_2_BLUE = new Rectangle(-34, -48, 12, 48);
    public static Rectangle VALID_X_SENSOR_READ_AREA_1_BLUE = new Rectangle(-48, 34, 48, 12);
    public static Rectangle VALID_X_SENSOR_READ_AREA_2_BLUE = new Rectangle(-48, -34, 48, 12);
}
