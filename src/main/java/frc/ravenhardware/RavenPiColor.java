package frc.ravenhardware;

import frc.robot.Constants;
import frc.util.PicoColorSensor;

public class RavenPiColor extends PicoColorSensor{
    public RavenPiColor() {
        super();
    }

    /**
     * Gets ball in front of sensor 0
     * @return 0 if undecided color / 1 if ball is red / 2 if ball is blue
     */
    public int getBallType0() {
        if(Math.abs(getRawColor0().red - getRawColor0().blue) > Constants.BALL_COLOR_THRESHOLD_0) {
            if(getRawColor0().red > getRawColor0().blue) {
                return(1);
            } else {
                return(2);
            }
        } else {
            return(0);
        }
    }

    /**
     * Gets ball in front of sensor 1
     * @return 0 if ball is undecided color / 1 if ball is red / 2 if ball is blue
     */
    public int getBallType1() {
        if(Math.abs(getRawColor1().red - getRawColor1().blue) > Constants.BALL_COLOR_THRESHOLD_1) {
            if(getRawColor1().red > getRawColor1().blue) {
                return(1);
            } else {
                return(2);
            }
        } else {
            return(0);
        }
    }
}
