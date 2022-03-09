package frc.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Deadband {
    public static Double adjustValueToZero(double value, double minimumValue) {
        if (value >= minimumValue || value <= (minimumValue * -1)) {
            return value;
        }

        return 0.0d;
    }

    public static double adjustValueRotation(double target, double actual, double maxDifferenceInRadians) {
        double fullCircle = Math.PI*2;
        double normalizedTarget = (target + fullCircle) % fullCircle;
        double normalizedActual = (actual + fullCircle) % fullCircle;

        double diff = Math.abs(normalizedTarget - normalizedActual);
        if (diff > maxDifferenceInRadians) {
            return target;
        }

        return actual;
    }
}
