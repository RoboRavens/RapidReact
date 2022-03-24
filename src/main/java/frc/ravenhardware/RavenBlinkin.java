package frc.ravenhardware;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * "ButtonCode" but for colors of LEDs, mixed with actual methods for setting the color.
 * 
 * IMPORTANT: ALL COLOR METHODS MUST BE CALLED USING periodic() OR A SIMILARILY FAST METHOD.
 * 
 */
public class RavenBlinkin {
    private Spark _blinkin;
    private Timer ledDelayer;
    private RavenBlinkinPatternCodes nextPatternState;
    private double _delayTime;

    // Non-color setters
    public RavenBlinkin(int PWM) {
        _blinkin = new Spark(PWM);
        ledDelayer = new Timer();
        nextPatternState = RavenBlinkinPatternCodes.BLINK_OFF;
        _delayTime = BlinkinCalibrations.DEFAULT_DELAY_TIME;
    }
    
    private boolean isDelayOver() {        
        if (ledDelayer.get() == 0.0) {
            ledDelayer.start();
        }
        if (ledDelayer.get() > _delayTime) {
            ledDelayer.stop();
            ledDelayer.reset();
            return true;
        }
        return false;
    }

    public void setBlinkDelay(double newDelay) {
        _delayTime = newDelay;
    }

    // Color methods

    /**
     * Sets the leds to a solid version of the input color
     * @param color BlinkinCalibrations color value to set
     */
    public void setSolid(double color) {
        _blinkin.set(color);
    }

    /**
     * Blinks the leds with the input color
     * @param color BlinkinCalibrations color value to blink at
     */
    public void setBlink(double color) {
        if (isDelayOver()) {
            if (nextPatternState == RavenBlinkinPatternCodes.BLINK_ON) {

                setSolid(color);
                nextPatternState = RavenBlinkinPatternCodes.BLINK_OFF;

            } else {

                setSolid(BlinkinCalibrations.OFF);
                nextPatternState = RavenBlinkinPatternCodes.BLINK_ON;
            }
        }
    }
}