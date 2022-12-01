package frc.ravenhardware;

import edu.wpi.first.wpilibj.AnalogInput;

public class PressureSensorTwo {
    
    public PressureSensorTwo(int i) {
    
    
    }

     AnalogInput analog = new AnalogInput(3);

        public double getPressure() {

           return analog.getVoltage();
        }

    }
