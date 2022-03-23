package frc.ravenhardware;

import frc.robot.Constants;
import frc.robot.lib.PicoColorSensor;

public class RavenPiColorSensor extends PicoColorSensor{
    public RavenPiColorSensor() {
        super();
    }

    /**
     * Gets ball in front of sensor of choice
     * @param pos The enum position of the sensor of choice: either ENTRY or EXIT
     * @return Returns an enum of the detected color, either UNKNOWN, RED, or BLUE
     */
    public RavenPiColor getBallType(RavenPiPosition pos) {
        RawColor colorval = pos == RavenPiPosition.ENTRY ? getRawColor0() : getRawColor1();
        double threshold = pos == RavenPiPosition.ENTRY ? Constants.BALL_COLOR_THRESHOLD_ENTRY : Constants.BALL_COLOR_THRESHOLD_EXIT;

        if(Math.abs(colorval.red - colorval.blue) < threshold) {
            return RavenPiColor.UNKNOWN; // Stop if difference in color is not trustworthy enough
        }

        return colorval.red > colorval.blue ? RavenPiColor.RED : RavenPiColor.BLUE;
    }
}
