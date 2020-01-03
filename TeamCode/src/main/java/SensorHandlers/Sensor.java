package SensorHandlers;

public interface Sensor {
    enum Type { LIDAR_SENSOR, NONE, TOUCH_SENSOR };

    Type getType();
    int getId();
    String getName();
    void kill();
}
