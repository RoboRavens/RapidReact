package frc.ravenhardware;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;


public class PressureSensor {
    AnalogPotentiometer pot = new AnalogPotentiometer(2, 200, 0);
   

  public PressureSensor(int i) {
    
    
    }
    

    public double getPressure() {

        return pot.get();

    }
}




