package SensorHandlers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LIDARSensor implements Sensor {
    DistanceSensor lidarSensor;
    int id;
    String name;

    public LIDARSensor() {

    }

    public LIDARSensor(DistanceSensor ls, String name) {
        this.name = name;
        lidarSensor = ls;
    }

    public LIDARSensor(DistanceSensor ls, int id) {
        this.id = id;
        lidarSensor = ls;
    }

    public LIDARSensor(DistanceSensor ls, int id, String name){
        this.id = id;
        this.name = name;
        lidarSensor = ls;
    }

    public void setSensor(DistanceSensor s) { this.lidarSensor = s; }
    public void setId(int id) { this.id = id; }
    public void setName(String n) { this.name = n; }

    public double getDistance(DistanceUnit d){
        return lidarSensor.getDistance(d);
    }

    @Override
    public Type getType() {
        return Type.LIDAR_SENSOR;
    }

    @Override
    public int getId() {
        return id;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public void kill() {

    }
}
