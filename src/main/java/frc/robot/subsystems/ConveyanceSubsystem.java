package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ConveyanceSubsystem extends SubsystemBase {
  
  private TalonSRX _conveyanceMotorOne;
  private DigitalInput _conveyanceTransitionBeamBreak;
  private DigitalInput _conveyanceIntakeBeamBreak;
  private boolean _isIndexing = false;

  public ConveyanceSubsystem() {
    _conveyanceMotorOne = new TalonSRX(RobotMap.CONVEYANCE_MOTOR);
    _conveyanceTransitionBeamBreak = new DigitalInput(RobotMap.CONVEYANCE_TRANSITION_BEAM_BREAK_CHANNEL);
    _conveyanceIntakeBeamBreak = new DigitalInput(RobotMap.CONVEYANCE_INTAKE_BREAM_BREAK_CHANNEL);
  }

  public void setConveyanceMaxReverse() {
    this.runConveyanceAtPercentPower(Constants.CONVEYANCE_ONE_FULL_SPEED_REVERSE);
  }

  public void setConveyanceMaxForward() {
    this.runConveyanceAtPercentPower(Constants.CONVEYANCE_ONE_FULL_SPEED);
  }

  public boolean isIndexing() {
    return _isIndexing;
  }

  public void setConveyanceIndexSpeedForward() {
    _isIndexing = true;
    this.runConveyanceAtPercentPower(Constants.CONVEYANCE_ONE_INDEX_SPEED);
  }

  public void setConveyanceNormalSpeedReverse() {
    this.runConveyanceAtPercentPower(Constants.CONVEYANCE_ONE_NORMAL_REVERSE_SPEED);
  }

  private void runConveyanceAtPercentPower(double magnitude) {
    this._conveyanceMotorOne.set(ControlMode.PercentOutput, magnitude);
  }

  public void stopConveyanceOne() {
    _isIndexing = false;
    this.runConveyanceAtPercentPower(Constants.CONVEYANCE_ONE_STOP);
  }

  public boolean getConveyanceHasBall() {
    return !_conveyanceTransitionBeamBreak.get();
  }

  public boolean getConveyanceHasBallNewFirstSensor() {
    return !_conveyanceIntakeBeamBreak.get();
  }

   @Override
   public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation 
  }

  public boolean getRobotHas2Balls() {
    return (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceHasBall() && Robot.FEEDER_SUBSYSTEM.getFeederHasBall());
  }

  public int getRobotCargoInventory() {
    int inventory = 0;

    if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceHasBall()) {
      inventory++;
    }

    if (Robot.FEEDER_SUBSYSTEM.getFeederHasBall()) {
      inventory++;
    }

    return inventory;
  }

  public int getRobotProperColorInventory() {
    int inventory = 0;

    


    return inventory;
  }
} 
