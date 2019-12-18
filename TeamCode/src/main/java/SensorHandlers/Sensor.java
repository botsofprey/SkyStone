package SensorHandlers;

public class Sensor<T> {
    private T type;
    private int id;

    public Sensor() {

    }

    public Sensor(Class<? extends T> type, int id) {
        this.type = (T)type;
        this.id = id;
    }

    public T getType() { return type; }
    public void setType(Class<? extends T> type) { this.type = (T)type; }
    public int getId() { return id; }
    public void setId(int id) { this.id = id; }
}
