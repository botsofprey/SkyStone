package Autonomous;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ConfigVariables {

    /*********************************
     * NOTE: (0, 0) is the center of the field
     * since we're using a half field, it is two squares to the right and 3 up
     *********************************/

    public static final double STONE_ONE_LEFT = 36.5;
    public static final double STONE_ONE_RIGHT = 30.5;
    public static final double STONE_TWO_LEFT = 26.5;
    public static final double STONE_TWO_RIGHT = 23.5;
    public static final double STONE_THREE_LEFT = 19;
    public static final double STONE_THREE_RIGHT = 15;

    public static final double DIST_STONE_ONE_LEFT = 38.5;
    public static final double DIST_STONE_ONE_RIGHT = 35;
    public static final double DIST_STONE_TWO_LEFT = 28.0;
    public static final double DIST_STONE_TWO_RIGHT = 28.5;
    public static final double DIST_STONE_THREE_LEFT = 23;
    public static final double DIST_STONE_THREE_RIGHT = 20;

    public static final double STONE_FOUR = 28.5;
    public static final double STONE_FIVE = 27;
    public static final double STONE_SIX = 19.5;

    public static final double DIST_STONE_FOUR = 11.3;
    public static final double DIST_STONE_FIVE = 5.3;
    public static final double DIST_STONE_SIX = 5;

    public static final Location UNDER_RED_BRIDGE = new Location(40, 0, 270);
    public static final Location UNDER_RED_BRIDGE_0_HEADING = new Location(40, 0, 0);
    public static final Location UNDER_RED_BRIDGE_BUILDING_ZONE = new Location(40, 10, 0);
    public static final Location UNDER_RED_BRIDGE_LOADING_ZONE = new Location(40, -10, 0);
    public static final Location BEHIND_RED_QUARRY = new Location(40, -38, 270);
    public static final Location FIRST_STONE_GROUP_CENTER_RED = new Location(24+16, -58.5, 270);
    public static final Location FIRST_STONE_GROUP_LEFT_RED = new Location(24+16, -66.5, 270);
    public static final Location FIRST_STONE_GROUP_RIGHT_RED = new Location(24+16, -50.5, 270);
    public static final Location SECOND_STONE_GROUP_CENTER_RED = new Location(24+16, -34.5, 270);
    public static final Location SECOND_STONE_GROUP_RIGHT_RED = new Location(24+16, -26.5, 270);
    public static final Location SECOND_STONE_GROUP_LEFT_RED = new Location(24+16, -42.5, 270); // should be 34.25, but robot stops too soon
    public static final Location RED_CALIBRATE_ZONE_1 = new Location(35, 35, 0);
    public static final Location RED_CALIBRATE_ZONE_2 = new Location(35, -35, 0);

    public static final Location FIRST_STONE_GROUP_CENTER_BLUE = new Location(-24-9, -38, 90);
    public static final Location FIRST_STONE_GROUP_LEFT_BLUE = new Location(-24-9, -32, 90);
    public static final Location FIRST_STONE_GROUP_RIGHT_BLUE = new Location(-24-9, -44, 90);
    public static final Location SECOND_STONE_GROUP_CENTER_BLUE = new Location(-24-9, -56, 90);
    public static final Location SECOND_STONE_GROUP_LEFT_BLUE = new Location(-24-9, -50, 90);
    public static final Location SECOND_STONE_GROUP_RIGHT_BLUE = new Location(-24-9, -62, 90);

    public static final Location RED_FOUNDATION_CENTER = new Location(24+11+6.5, 49.25, 270);
    public static final Location RED_FOUNDATION_STACK_CENTER = new Location(32, 49.25, 270);
    public static final Location RED_FOUNDATION_LEFT = new Location(39.5, 37.5, 270);
    public static final Location RED_FOUNDATION_STACK_LEFT = new Location(20+9+4, 37.5, 270);
    public static final Location BLUE_FOUNDATION_CENTER = new Location(-24-10-5, 49.5, 90);

    // TODO check these locations
    public static final Location RED_ZONE_ONE = new Location(90, 30);
    public static final Location RED_ZONE_TWO = new Location(60, 55);
    public static final Location RED_ZONE_THREE = new Location(60, 60);
    public static final Location CENTER = new Location(0,0);

    public static final Location STARTING_RING_PILE = new Location(30, -60);

    public static final HorizontalLine SHOOT_LINE = new HorizontalLine(new Location(-120, 30), 240);

    public static final Location RED_WOBBLE_GOAL_LEFT = new Location(24, -48);
    public static final Location RED_WOBBLE_GOAL_RIGHT = new Location(48, -48);

    public static final Location STARTING_ROBOT_LOCATION_LEFT = new Location(24, -63, 0);
  //  public static final Location STARTING_ROBOT_LOCATION_LEFT = new Location(0, -10, 0);
    public static final Location STARTING_ROBOT_LOCATION_RIGHT = new Location(60, -165, 0);

    // TODO find these locations
    public static final Location POWER_SHOT_LEFT = new Location(0, 0);
    public static final Location POWER_SHOT_MIDDLE = new Location(0, 0);
    public static final Location POWER_SHOT_RIGHT = new Location(0, 0);
    public static final double POWER_SHOT_HEIGHT_CM = 0;

    public static final Location TOP_GOAL = new Location(0, 0);
    public static final double TOP_GOAL_HEIGHT_CM = 0;

    public static final Rectangle VALID_Y_SENSOR_READ_AREA_1_RED = new Rectangle(34, 48, 12, 48);
    public static final Rectangle VALID_Y_SENSOR_READ_AREA_2_RED = new Rectangle(34, -48, 12, 48);
    public static final Rectangle VALID_X_SENSOR_READ_AREA_1_RED = new Rectangle(48, 34, 48, 12);
    public static final Rectangle VALID_X_SENSOR_READ_AREA_2_RED = new Rectangle(48, -34, 48, 12);

    public static final Rectangle VALID_Y_SENSOR_READ_AREA_1_BLUE = new Rectangle(-34, 48, 12, 48);
    public static final Rectangle VALID_Y_SENSOR_READ_AREA_2_BLUE = new Rectangle(-34, -48, 12, 48);
    public static final Rectangle VALID_X_SENSOR_READ_AREA_1_BLUE = new Rectangle(-48, 34, 48, 12);
    public static final Rectangle VALID_X_SENSOR_READ_AREA_2_BLUE = new Rectangle(-48, -34, 48, 12);
}
