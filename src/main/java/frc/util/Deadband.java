package frc.util;

public class Deadband {
    public static Double adjustValue(double value, double minimumValue) {
        if (value >= minimumValue || value <= (minimumValue * -1)) {
            return value;
        }

        return 0.0d;
    }
}
