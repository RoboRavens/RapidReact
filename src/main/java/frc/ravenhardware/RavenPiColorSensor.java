package frc.ravenhardware;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.PicoColorSensor;

public class RavenPiColorSensor extends PicoColorSensor{
    private boolean _colorSensorFeatureEnabled = true;

    public RavenPiColorSensor() {
        super();
    }

    /**
     * Gets ball in front of sensor of choice
     * @param sensorPosition The enum position of the sensor of choice: either CONVEYANCE or FEEDER
     * @return Returns an enum of the detected color, either Invalid, Red, or Blue
     */
    public Alliance getSensorBallColor(RavenPiPosition sensorPosition) {
        RawColor colorval;
        double threshold;

        if(sensorPosition == RavenPiPosition.CONVEYANCE) {
            colorval = getRawColor0();
        } else {
            colorval = getRawColor1();
        }

        if(sensorPosition == RavenPiPosition.CONVEYANCE) {
            threshold = Constants.BALL_COLOR_THRESHOLD_ENTRY;
        } else {
            threshold = Constants.BALL_COLOR_THRESHOLD_EXIT;
        }

        if (!(colorval.red > threshold || colorval.blue > threshold)) { //NOR gate
            return Alliance.Invalid; // Return invalid if both red and blue are below threshold
        }

        return colorval.red > colorval.blue ? Alliance.Red : Alliance.Blue;
    }

    public RawColor getFirstSensorRawColor() {
        return getRawColor0();
    }

    public RawColor getSecondSensorRawColor() {
        return getRawColor1();
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

        if (_colorSensorFeatureEnabled == false) {
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

        if (ballColor == Robot.ALLIANCE_COLOR) {
            getIsCorrectBallType = true;
        }

        if (_colorSensorFeatureEnabled == false) {
            getIsCorrectBallType = true;
        }

        return getIsCorrectBallType;
    }

    public void setColorSensorFeatureEnabled(boolean value) {
        _colorSensorFeatureEnabled = value;
    }
}
