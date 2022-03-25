package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ravenhardware.RavenPiPosition;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ConveyanceSubsystem extends SubsystemBase {
  
  private TalonSRX _conveyanceMotorOne;
  private DigitalInput _conveyanceStagingBeamBreak;
  private DigitalInput _conveyanceIntakeBeamBreak;
  private boolean _isIndexing = false;

  public ConveyanceSubsystem() {
    _conveyanceMotorOne = new TalonSRX(RobotMap.CONVEYANCE_MOTOR);
    _conveyanceStagingBeamBreak = new DigitalInput(RobotMap.CONVEYANCE_TRANSITION_BEAM_BREAK_CHANNEL);
    _conveyanceIntakeBeamBreak = new DigitalInput(RobotMap.CONVEYANCE_INTAKE_BREAM_BREAK_CHANNEL);
  }

  public void setConveyanceEjectCargo() {
    this.runConveyanceAtPercentPower(Constants.CONVEYANCE_ONE_FULL_SPEED_REVERSE);
  }

  public void setConveyanceCollectCargo() {
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

  public boolean getConveyanceStagingBeamBreakHasBall() {
    return !_conveyanceStagingBeamBreak.get();
  }

  public boolean getConveyanceIntakeBeamBreakHasBall() {
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

  public boolean conveyanceHasProperColorCargo() {
    boolean conveyanceHasProperColorCargo = false;
    
    if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceStagingBeamBreakHasBall()) {
      if (Robot.COLOR_SENSOR.getSensorIsCorrectBallColorLenient(RavenPiPosition.CONVEYANCE)) {
        conveyanceHasProperColorCargo = true;
      }
    }

    return conveyanceHasProperColorCargo;
  }

  public boolean conveyanceHasWrongColorCargo() {
    boolean conveyanceHasWrongColorCargo = false;
    
    if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceStagingBeamBreakHasBall()) {
      if (Robot.COLOR_SENSOR.getSensorIsCorrectBallColorLenient(RavenPiPosition.CONVEYANCE)) {
        conveyanceHasWrongColorCargo = true;
      }
    }

    return conveyanceHasWrongColorCargo;
  }
} 
