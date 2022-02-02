
package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ConveyanceSubsystem extends SubsystemBase {
  
    private TalonFX _conveyanceMotorOne;
 
 
 
   public ConveyanceSubsystem() {
    _conveyanceMotorOne = new TalonFX(RobotMap.CONVEYANCE_MOTOR_ONE);
    
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
