package frc.ravenhardware;

import edu.wpi.first.wpilibj.AnalogInput;

public class PressureSensorTwo {
    
   //this only would work with doubles so I just implemented them as a double variable
   public double supplyVoltage = 5;
   public double x = 250;
   
   
   public PressureSensorTwo(int i) {
     
   
    
    }

     AnalogInput analog = new AnalogInput(3);
     public double P = analog.getValue();
        
     public double getPressure() {
         
         return P = analog.getVoltage() / supplyVoltage * x - 25;
         //CALCULATING PRESSURE
         //https://www.revrobotics.com/content/docs/REV-11-1107-DS.pdf
         
      }
       
}
