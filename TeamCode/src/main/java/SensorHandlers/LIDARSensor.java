package SensorHandlers;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static SensorHandlers.SensorPackage.LIDAR_SENSOR;

public class LIDARSensor extends Sensor {
    LIDARSensor lidarSensor;

    public LIDARSensor() {
        super(new LIDARSensor(),LIDAR_SENSOR);
    }

    public LIDARSensor(LIDARSensor ls){
        super(new LIDARSensor(), LIDAR_SENSOR);
        lidarSensor = ls;
    }

    public double getDistance(DistanceUnit d){
        return lidarSensor.getDistance(d);
    }
}
