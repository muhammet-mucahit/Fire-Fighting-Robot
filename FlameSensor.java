import com.ridgesoft.intellibrain.IntelliBrain;
import com.ridgesoft.robotics.AnalogInput;

public class FlameSensor {

    private final int SENSOR_COUNT = 5;
    private static AnalogInput[] sensors;

    public int flameVal;
    public int flameDir;

    private static int flamePresentLimit;
    private static int flameNearLimit;

    public FlameSensor(int limit1, int limit2) {
        flamePresentLimit = limit1;
        flameNearLimit = limit2;
        sensors = new AnalogInput[SENSOR_COUNT];
        for (int i = 0; i < SENSOR_COUNT; i++)
            sensors[i] = IntelliBrain.getAnalogInput(i + 1);
    }

    public int scan() {
        flameVal = 0;

        for (int i = 0; i < SENSOR_COUNT; i++) {
            int sampleOfSensor = sensors[i].sample();
            if (sampleOfSensor > flameVal) {
                flameVal = sampleOfSensor;
                flameDir = i + 1;
            }
        }

        if (flameVal < flamePresentLimit)
            flameDir = -1;

        // It is alternative to stop in front of the candle. Third way.
        if (flameVal > flameNearLimit)
            flameDir = 0;

        return flameDir;
    }

    public int getSensorVaue(int s) {
        return sensors[s].sample();
    }
}
