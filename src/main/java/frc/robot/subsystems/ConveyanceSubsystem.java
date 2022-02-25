
package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ravenhardware.BufferedDigitalInput;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class ConveyanceSubsystem extends SubsystemBase {
  
    private TalonSRX _conveyanceMotorOne;
    private DigitalInput _conveyanceSensorA;
 
 
 
   public ConveyanceSubsystem() {
    _conveyanceMotorOne = new TalonSRX(RobotMap.CONVEYANCE_MOTOR);
    _conveyanceSensorA = new DigitalInput(RobotMap.SENSOR_A_CHANNEL);
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
    this._conveyanceMotorOne.set(ControlMode.PercentOutput, magnitude);
    System.out.println("Attempting to run conveyance run at 1");
  }

  public void stopConveyanceOne() {
    this.runConveyanceAtPercentPower(Constants.CONVEYANCE_ONE_STOP);
  }

  public boolean getConveyanceOneSubsystemHasBall() {
    return !_conveyanceSensorA.get();
  }

  public void defaultCommand() {
    //this.stopConveyanceOne();
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













