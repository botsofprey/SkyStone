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

    public static Location FIRST_STONE_GROUP_CENTER_RED = new Location(24+9, -56, 270);
    public static Location FIRST_STONE_GROUP_LEFT_RED = new Location(24+9, -50, 270);
    public static Location FIRST_STONE_GROUP_RIGHT_RED = new Location(24+9, -62, 270);
    public static Location SECOND_STONE_GROUP_CENTER_RED = new Location(24+9, -38, 270);
    public static Location SECOND_STONE_GROUP_LEFT_RED = new Location(24+9, -32, 270);
    public static Location SECOND_STONE_GROUP_RIGHT_RED = new Location(24+9, -44, 270);

    public static Location FIRST_STONE_GROUP_CENTER_BLUE = new Location(-24-9, -38, 90);
    public static Location FIRST_STONE_GROUP_LEFT_BLUE = new Location(-24-9, -32, 90);
    public static Location FIRST_STONE_GROUP_RIGHT_BLUE = new Location(-24-9, -44, 90);
    public static Location SECOND_STONE_GROUP_CENTER_BLUE = new Location(-24-9, -56, 90);
    public static Location SECOND_STONE_GROUP_LEFT_BLUE = new Location(-24-9, -50, 90);
    public static Location SECOND_STONE_GROUP_RIGHT_BLUE = new Location(-24-9, -62, 90);

    public static Location RED_FOUNDATION_CENTER = new Location(23.75+9, 49.75, 270);
    public static Location BLUE_FOUNDATION_CENTER = new Location(-23.75-9, 49.75, 90);
}
