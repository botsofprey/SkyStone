package SensorHandlers;

import android.support.annotation.NonNull;
import android.support.annotation.Nullable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

public class SensorPackage {
    private List<Sensor> sensors = new ArrayList<>();

    public SensorPackage(Sensor... sensors) { this.sensors.addAll(Arrays.asList(sensors)); }

    public <T> T getSensor(Class<? extends T> sensorType, int id) {
        for(Sensor s : sensors) {
            if(s.getType() == sensorType.getClass() && s.getId() == id) return (T)s.getType();
        }
        return null;
    }
}