
package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class ConveyanceSubsystem extends SubsystemBase {
  
    private TalonSRX _conveyanceMotorOne;
 
 
 
   public ConveyanceSubsystem() {
    _conveyanceMotorOne = new TalonSRX(RobotMap.CONVEYANCE_MOTOR_ONE);
    
  }

 
  




  public void setConveyanceMaxReverse() {
    this.runConveyanceAtPercentPower(Constants.CONVEYANCE_ONE_FULL_SPEED_REVERSE);
  }

  public void setConveyanceMaxForward() {
    this.runConveyanceAtPercentPower(Constants.CONVEYANCE_ONE_FULL_SPEED);
  }

  public void setConveyanceNormalSpeedForward() {
    this.runConveyanceAtPercentPower(Constants.CONVEYANCE_ONE_NORMAL_SPEED);
  }

  public void setConveyanceNormalSpeedReverse() {
    this.runConveyanceAtPercentPower(Constants.CONVEYANCE_ONE_NORMAL_REVERSE_SPEED);
  }

  private void runConveyanceAtPercentPower(double magnitude) {
    _conveyanceMotorOne.set(ControlMode.PercentOutput, magnitude);
  }

  public void stopConveyanceOne() {
    this.runConveyanceAtPercentPower(Constants.CONVEYANCE_ONE_STOP);
  }

  public void defaultCommand() {
    this.stopConveyanceOne();
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
