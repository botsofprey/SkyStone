package SensorHandlers;

import java.util.ArrayList;
import java.util.Arrays;

public class SensorPackage {
    public static final int LIDAR_SENSOR = 0;
    private ArrayList<Sensor> sensors = new ArrayList<>();

    public SensorPackage(Sensor... sensors) { this.sensors.addAll(Arrays.asList(sensors)); }

    public <T extends Sensor> T getSensor(T sensorType, int id) {
        for(Sensor s : sensors) {
            if(s.getType() == sensorType && s.getId() == id) return (T)s.getType();
        }
        return null;
    }
}