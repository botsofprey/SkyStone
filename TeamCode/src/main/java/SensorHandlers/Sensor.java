package SensorHandlers;

public class Sensor<T extends Sensor> {
    private T type;
    private int id;

    public Sensor() {

    }

    public Sensor(T type, int id) {
        this.type = type;
        this.id = id;
    }

    public <T extends Sensor> T getType() { return (T) type; }
    public void setType(T type) { this.type = type; }
    public int getId() { return id; }
    public void setId(int id) { this.id = id; }
}
