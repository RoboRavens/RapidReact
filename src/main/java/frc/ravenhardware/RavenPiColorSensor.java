package frc.ravenhardware;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.PicoColorSensor;

public class RavenPiColorSensor extends PicoColorSensor{
    private boolean _colorSensorFeatureEnabled = true;
    private BufferedBoolean _feederSensorIsWrongBallColorStrict;

    public RavenPiColorSensor() {
        super();
        _feederSensorIsWrongBallColorStrict = new BufferedBoolean(() -> this.getFeederSensorIsWrongBallColorStrictInternal(), 15, true, false);
    }

    /**
     * Gets ball in front of sensor of choice
     * @param sensorPosition The enum position of the sensor of choice: either CONVEYANCE or FEEDER
     * @return Returns an enum of the detected color, either Invalid, Red, or Blue
     */
    /*
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
    */

    public void robotPeriodic() {
        _feederSensorIsWrongBallColorStrict.maintainState();
    }

    public Alliance getIntakeSensorAllianceColor() {
        Alliance ballColor = Alliance.Invalid;

        RawColor sensorColor = getRawColor0();

        if ((sensorColor.red < 450 || sensorColor.green > sensorColor.red * 2) && sensorColor.green > 110 && sensorColor.blue > 70) {
            ballColor = Alliance.Blue;
        }
        else if (sensorColor.red > 130) {
            ballColor = Alliance.Red;
        }

        return ballColor;
    }

    public Alliance getFeederSensorAllianceColor() {
        Alliance ballColor = Alliance.Invalid;

        RawColor sensorColor = getRawColor1();

        if (sensorColor.red < 230 && sensorColor.green > 90 && sensorColor.blue > 50) {
            ballColor = Alliance.Blue;
        }
        else if (sensorColor.red > 50 && sensorColor.red > sensorColor.green * 2) {
            ballColor = Alliance.Red;
        }

        return ballColor;
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
    /*
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
    */

    /**
     * Determines whether the ball in the given sensor location matches the correct color.
     * Strict mode - returns FALSE if the sensor can't determine the color.
     * @param sensorPosition The enum position of the sensor of choice: either CONVEYANCE or FEEDER
     * @return Returns an enum of the detected color, either Invalid, Red, or Blue
     */
    /*
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
    */

    public void setColorSensorFeatureEnabled(boolean value) {
        _colorSensorFeatureEnabled = value;
    }

    public boolean getColorSensorFeatureEnabled() {
        return _colorSensorFeatureEnabled;
    }

    public boolean getConveyanceSensorIsCorrectBallColorLenient() {
        boolean getIsCorrectBallType = false;

        Alliance ballColor = getIntakeSensorAllianceColor();

        if (ballColor == Robot.ALLIANCE_COLOR || ballColor == Alliance.Invalid) {
            getIsCorrectBallType = true;
        }

        if (_colorSensorFeatureEnabled == false) {
            getIsCorrectBallType = true;
        }

        return getIsCorrectBallType;
    }

    public boolean getConveyanceSensorIsCorrectBallColorStrict() {
        boolean getIsCorrectBallType = false;

        Alliance ballColor = getIntakeSensorAllianceColor();

        if (ballColor == Robot.ALLIANCE_COLOR) {
            getIsCorrectBallType = true;
        }

        if (_colorSensorFeatureEnabled == false) {
            getIsCorrectBallType = true;
        }

        return getIsCorrectBallType;
    }

    public boolean getConveyanceSensorIsWrongBallColorStrict() {
        boolean getIsWrongBallType = false;

        Alliance ballColor = getIntakeSensorAllianceColor();

        if (ballColor != Robot.ALLIANCE_COLOR && ballColor != Alliance.Invalid) {
            getIsWrongBallType = true;
        }

        if (_colorSensorFeatureEnabled == false) {
            getIsWrongBallType = false;
        }

        return getIsWrongBallType;
    }

    private boolean getFeederSensorIsWrongBallColorStrictInternal() {
        boolean getIsWrongBallType = false;

        Alliance ballColor = getFeederSensorAllianceColor();

        if (ballColor != Robot.ALLIANCE_COLOR && ballColor != Alliance.Invalid) {
            getIsWrongBallType = true;
        }

        if (_colorSensorFeatureEnabled == false) {
            getIsWrongBallType = false;
        }

        return getIsWrongBallType;
    }

    public boolean getFeederSensorIsWrongBallColorStrict() {
        return _feederSensorIsWrongBallColorStrict.get();
    }

    
    public boolean getFeederSensorIsCorrectBallColorLenient() {
        boolean getIsCorrectBallType = false;

        Alliance ballColor = getFeederSensorAllianceColor();

        if (ballColor == Robot.ALLIANCE_COLOR || ballColor == Alliance.Invalid) {
            getIsCorrectBallType = true;
        }

        if (_colorSensorFeatureEnabled == false) {
            getIsCorrectBallType = true;
        }

        return getIsCorrectBallType;
    }

    public boolean getFeederSensorIsCorrectBallColorStrict() {
        boolean getIsCorrectBallType = false;

        Alliance ballColor = getFeederSensorAllianceColor();

        if (ballColor == Robot.ALLIANCE_COLOR) {
            getIsCorrectBallType = true;
        }

        if (_colorSensorFeatureEnabled == false) {
            getIsCorrectBallType = true;
        }

        return getIsCorrectBallType;
    }
}
