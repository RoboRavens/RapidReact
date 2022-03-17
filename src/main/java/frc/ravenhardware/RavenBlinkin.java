package frc.ravenhardware;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.ravenhardware.BlinkinCalibrations;

/**
 * "ButtonCode" but for colors of LEDs, mixed with acutal methods for setting the color.
 * 
 * IMPORTANT: ALL COLOR METHODS MUST BE CALLED USING periodic() OR A SIMILARILY FAST METHOD.
 * 
 */
public class RavenBlinkin {

    private static Spark _blinkin;
    private Timer ledDelayer;
    private RavenBlinkinPatternCodes nextPatternState;

    // Non-color setters
    private void initializeBlinkin(int PWM) {
        _blinkin = new Spark(PWM);
        ledDelayer = new Timer();
        nextPatternState = RavenBlinkinPatternCodes.SOLID_OFF;
    }
    
    public RavenBlinkin(int PWM) {
        initializeBlinkin(PWM);
    }
    
    private boolean isDelayOver() {        
        if (ledDelayer.get() == 0.0) {
            ledDelayer.start();
        }
        if (ledDelayer.get() > BlinkinCalibrations.DELAY_TIME) {
            ledDelayer.stop();
            ledDelayer.reset();
            return true;
        }
        return false;
    }

    // Color methods
  
    public void solidOff() {
        _blinkin.set(BlinkinCalibrations.LED_OFF);
    }

    public void blinkGreen() {
        if (isDelayOver()) {
            if (nextPatternState.equals(RavenBlinkinPatternCodes.SOLID_GREEN)) {
                solidGreen();
                nextPatternState = RavenBlinkinPatternCodes.SOLID_OFF;

            } else {
                solidOff();
                nextPatternState = RavenBlinkinPatternCodes.SOLID_GREEN;

            }
        }
    }

    public static void solidGreen() {
        _blinkin.set(BlinkinCalibrations.SOLID_GREEN);
    }


    public void blinkYellow() {
        if (isDelayOver()) {
            if (nextPatternState.equals(RavenBlinkinPatternCodes.SOLID_YELLOW)) {

                solidYellow();
                nextPatternState = RavenBlinkinPatternCodes.SOLID_OFF;

            } else {

                solidOff();
                nextPatternState = RavenBlinkinPatternCodes.SOLID_YELLOW;

            }
        }
    }

    public static void solidYellow() {
        _blinkin.set(BlinkinCalibrations.SOLID_YELLOW);
    }


    public static void solidRed() {
        _blinkin.set(BlinkinCalibrations.SOLID_RED);
    }

    public void blinkRed() {
        if (isDelayOver()) {
            if (nextPatternState.equals(RavenBlinkinPatternCodes.SOLID_RED)) {

                solidBlue();
                nextPatternState = RavenBlinkinPatternCodes.SOLID_OFF;

            } else {

                solidOff();
                nextPatternState = RavenBlinkinPatternCodes.SOLID_RED;

            }
        }
    }

    public void blinkBlue() {
        if (isDelayOver()) {
            if (nextPatternState.equals(RavenBlinkinPatternCodes.SOLID_BLUE)) {

                solidBlue();
                nextPatternState = RavenBlinkinPatternCodes.SOLID_OFF;

            } else {

                solidOff();
                nextPatternState = RavenBlinkinPatternCodes.SOLID_BLUE;

            }
        }
    }

    public void solidBlue() {
        _blinkin.set(BlinkinCalibrations.SOLID_BLUE);
    }


    public void blinkWhite() {
        if (isDelayOver()) {
            if (nextPatternState.equals(RavenBlinkinPatternCodes.SOLID_WHITE)) {

                solidWhite();
                nextPatternState = RavenBlinkinPatternCodes.SOLID_OFF;

            } else {

                solidOff();
                nextPatternState = RavenBlinkinPatternCodes.SOLID_WHITE;

            }
        }
    }
    
    public void solidWhite() {
        _blinkin.set(BlinkinCalibrations.SOLID_WHITE);
    }

   
}