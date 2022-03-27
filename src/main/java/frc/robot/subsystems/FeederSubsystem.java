// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ravenhardware.RavenPiPosition;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class FeederSubsystem extends SubsystemBase {
   
  private TalonFX _conveyanceMotorTwo;
  private TalonFX _feederWheelMotor;
  private DigitalInput _feederBeamBreak;


  public FeederSubsystem() {
      _conveyanceMotorTwo = new TalonFX(RobotMap.FEEDER_CONVEYANCE_MOTOR);
      _feederWheelMotor = new TalonFX(RobotMap.FEEDER_MOTOR);
      _feederBeamBreak = new DigitalInput(RobotMap.FEEDER_BEAM_BREAK_CHANNEL);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }



  public void setConveyanceMagnitude() {

  }

  public void setConveyanceMaxReverse() {
    this.runConveyanceAtPercentPower(Constants.CONVEYANCE_TWO_FULL_SPEED_REVERSE);
  }

  public void setConveyanceTwoMaxForward() {
    if (Robot.CONVEYANCE_SUBSYSTEM.getIsIndexingFromStagingToFeeder()) {
      this.runConveyanceAtPercentPower(Constants.CONVEYANCE_TWO_SPEED_WHILE_INDEXING);
    } else {
      this.runConveyanceAtPercentPower(Constants.CONVEYANCE_TWO_FULL_SPEED);
    }
  }

  public void setConveyanceNormalSpeedForward() {
    this.runConveyanceAtPercentPower(Constants.CONVEYANCE_TWO_NORMAL_SPEED);
  }

  public void setConveyanceNormalSpeedReverse() {
    this.runConveyanceAtPercentPower(Constants.CONVEYANCE_TWO_NORMAL_REVERSE_SPEED);
  }

  private void runConveyanceAtPercentPower(double magnitude) {
    _conveyanceMotorTwo.set(ControlMode.PercentOutput, magnitude);
  }

  public void conveyanceStop() {
    this.runConveyanceAtPercentPower(Constants.CONVEYANCE_TWO_STOP);
  }

  public void defaultCommand() {
    this.conveyanceStop();
  }

  private void runWheelAtPercentPower(double magnitude) {
    _feederWheelMotor.set(ControlMode.PercentOutput, magnitude);
  }

  public void feederWheelForward() {
    this.runWheelAtPercentPower(Constants.CONVEYANCE_TWO_FEEDER_SPEED);
  }

  public void feederWheelStop() {
    this.runWheelAtPercentPower(Constants.CONVEYANCE_TWO_FEEDER_STOP);
  }

  public void feederWheelReverse() {
    this.runWheelAtPercentPower(Constants.CONVEYANCE_TWO_REVERSE_FEEDER);
  }

  public boolean getFeederHasBall() {
    return !_feederBeamBreak.get();
  }

  // This method runs the feeder wheel and the conveyance, but only
  // if the shooter is at an appropriate RPM.
  public void shoot() {
    if (Robot.SHOOTER_SUBSYSTEM.motorsAreRecovered()) {
      feederWheelForward();
      setConveyanceTwoMaxForward();
    }
    else {
      feederWheelStop();
      conveyanceStop();
    }
  }

  public void stopFeederAndConveyance() {
    this.feederWheelStop();
    this.conveyanceStop();
  }

  public void forceShoot() {
    feederWheelForward();
    setConveyanceTwoMaxForward();
  }

  public void forceShootTeleop() {
    if (Robot.SHOOTER_SUBSYSTEM.motorsAreSpinning()) {
      feederWheelForward();
      setConveyanceTwoMaxForward();
    }
    else {
      feederWheelStop();
      conveyanceStop();
    }
  }

  public boolean feederHasProperColorCargo() {
    boolean feederHasProperColorCargo = false;
    
    if (Robot.FEEDER_SUBSYSTEM.getFeederHasBall()) {
      if (Robot.COLOR_SENSOR.getSensorIsCorrectBallColorLenient(RavenPiPosition.FEEDER)) {
        feederHasProperColorCargo = true;
      }
    }

    return feederHasProperColorCargo;
  }

  public boolean feederHasWrongColorCargo() {
    boolean feederHasWrongColorCargo = false;
    
    if (Robot.FEEDER_SUBSYSTEM.getFeederHasBall()) {
      if (Robot.COLOR_SENSOR.getSensorIsCorrectBallColorLenient(RavenPiPosition.FEEDER)) {
        feederHasWrongColorCargo = true;
      }
    }

    return feederHasWrongColorCargo;
  }
}
