package frc.ravenhardware;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.PicoColorSensor;

public class RavenPiColorSensor extends PicoColorSensor{
    public RavenPiColorSensor() {
        super();
    }

    /**
     * Gets ball in front of sensor of choice
     * @param sensorPosition The enum position of the sensor of choice: either CONVEYANCE or FEEDER
     * @return Returns an enum of the detected color, either Invalid, Red, or Blue
     */
    public Alliance getSensorBallColor(RavenPiPosition sensorPosition) {
        RawColor colorval = sensorPosition == RavenPiPosition.CONVEYANCE ? getRawColor0() : getRawColor1();
        double threshold = sensorPosition == RavenPiPosition.CONVEYANCE ? Constants.BALL_COLOR_THRESHOLD_ENTRY : Constants.BALL_COLOR_THRESHOLD_EXIT;

        if (Math.abs(colorval.red - colorval.blue) < threshold) {
            return Alliance.Invalid; // Stop if difference in color is not trustworthy enough
        }

        return colorval.red > colorval.blue ? Alliance.Red : Alliance.Blue;
    }

    /**
     * Determines whether the ball in the given sensor location matches the correct color.
     * Lenient mode - returns TRUE if the sensor can't determine the color.
     * @param sensorPosition The enum position of the sensor of choice: either CONVEYANCE or FEEDER
     * @return Returns an enum of the detected color, either Invalid, Red, or Blue
     */
    public boolean getSensorIsCorrectBallColorLenient( RavenPiPosition sensorPosition) {
        boolean getIsCorrectBallType = false;

        Alliance ballColor = getSensorBallColor(sensorPosition);

        if (ballColor == Robot.ALLIANCE_COLOR || ballColor == Alliance.Invalid) {
            getIsCorrectBallType = true;
        }

        return getIsCorrectBallType;
    }

    /**
     * Determines whether the ball in the given sensor location matches the correct color.
     * Strict mode - returns FALSE if the sensor can't determine the color.
     * @param sensorPosition The enum position of the sensor of choice: either CONVEYANCE or FEEDER
     * @return Returns an enum of the detected color, either Invalid, Red, or Blue
     */
    public boolean getSensorIsCorrectBallColorStrict(RavenPiPosition sensorPosition) {
        boolean getIsCorrectBallType = false;

        Alliance ballColor = getSensorBallColor(sensorPosition);

        if (ballColor == Robot.ALLIANCE_COLOR || ballColor == Alliance.Invalid) {
            getIsCorrectBallType = true;
        }

        return getIsCorrectBallType;
    }
}
