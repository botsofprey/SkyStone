package SensorHandlers;

public class Sensor<T> {
    private Class<? extends T> type;
    private int id;

    public Sensor() {

    }

    public Sensor(Class<? extends T> type, int id) {
        this.type = type;
        this.id = id;
    }

    public Class<? extends T> getType() { return type; }
    public void setType(Class<? extends T> type) { this.type = type; }
    public int getId() { return id; }
    public void setId(int id) { this.id = id; }
}
