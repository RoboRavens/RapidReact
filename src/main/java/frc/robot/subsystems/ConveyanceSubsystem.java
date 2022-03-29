package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ravenhardware.RavenPiPosition;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.util.ConveyanceState;

public class ConveyanceSubsystem extends SubsystemBase {
  
  private WPI_TalonSRX _conveyanceMotorOne;
  private DigitalInput _conveyanceStagingBeamBreak;
  private DigitalInput _conveyanceIntakeBeamBreak;
  private boolean _isIndexingFromStagingToFeeder = false;
  private boolean _isIndexingFromEntranceToStaging = false;
  private ConveyanceState _conveyanceState = ConveyanceState.OFF;

  public ConveyanceSubsystem() {
    _conveyanceMotorOne = new WPI_TalonSRX(RobotMap.CONVEYANCE_MOTOR);
    _conveyanceStagingBeamBreak = new DigitalInput(RobotMap.CONVEYANCE_TRANSITION_BEAM_BREAK_CHANNEL);
    _conveyanceIntakeBeamBreak = new DigitalInput(RobotMap.CONVEYANCE_INTAKE_BREAM_BREAK_CHANNEL);
  }

  public void setIsIndexingFromEntranceToStaging(boolean isIndexing) {
    _isIndexingFromEntranceToStaging = isIndexing;
  }

  public boolean getIsIndexingFromEntranceToStaging() {
    return _isIndexingFromEntranceToStaging;
  }

  public void setConveyanceEjectCargo() {
    this.runConveyanceAtVoltage(Constants.CONVEYANCE_ONE_FULL_SPEED_REVERSE);
    _conveyanceState = ConveyanceState.EJECTING;
  }

  public void setConveyanceIntakeCargo() {
    this.runConveyanceAtVoltage(Constants.CONVEYANCE_ONE_FULL_SPEED);
    _conveyanceState = ConveyanceState.INTAKING;
  }

  public boolean getIsIndexingFromStagingToFeeder() {
    return _isIndexingFromStagingToFeeder;
  }

  public void setConveyanceIndexCargoForward() {
    _isIndexingFromStagingToFeeder = true;
    this.runConveyanceAtVoltage(Constants.CONVEYANCE_ONE_INDEX_SPEED);
    _conveyanceState = ConveyanceState.INDEXING;
  }

  public ConveyanceState getConveyanceState() {
    return _conveyanceState;
  }

  /*
  Method is not being used as of Mar 28 2022
  public void setConveyanceNormalSpeedReverse() {
    this.runConveyanceAtVoltage(Constants.CONVEYANCE_ONE_NORMAL_REVERSE_SPEED);
  }
  */

  private void runConveyanceAtVoltage(double magnitude) {
    // this._conveyanceMotorOne.set(ControlMode.PercentOutput, magnitude);
    
    // The better way to do this would be to update the constant values on
    // the methods that call this, but until we know it works I don't want
    // to change all the code to do that so we'll just do the conversion here.
    double voltage = magnitude * 12.0;
    this._conveyanceMotorOne.setVoltage(voltage);
  }

  public void stopConveyanceOne() {
    _isIndexingFromStagingToFeeder = false;
    this.runConveyanceAtVoltage(Constants.CONVEYANCE_ONE_STOP);
  }

  public boolean getConveyanceStagingBeamBreakHasBall() {
    return !_conveyanceStagingBeamBreak.get();
  }

  public boolean getConveyanceEntryBeamBreakHasBall() {
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
      if (Robot.COLOR_SENSOR.getSensorIsCorrectBallColorStrict(RavenPiPosition.CONVEYANCE)) {
        conveyanceHasWrongColorCargo = true;
      }
    }

    return conveyanceHasWrongColorCargo;
  }
} 
