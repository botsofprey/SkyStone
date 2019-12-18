package SensorHandlers;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LIDARSensor extends Sensor {
    DistanceSensor lidarSensor;

    public LIDARSensor() {
        super();
        setType(LIDARSensor.class);
    }

    public LIDARSensor(DistanceSensor ls, int id){
        super(LIDARSensor.class, id);
        lidarSensor = ls;
    }

    public double getDistance(DistanceUnit d){
        return lidarSensor.getDistance(d);
    }
}
