// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class FeederSubsystem extends SubsystemBase {
   
  private TalonFX _conveyanceMotor;
  private TalonFX _feederWheelMotor;


  /** Creates a new ExampleSubsystem. */
  public FeederSubsystem() {
      _conveyanceMotor = new TalonFX(RobotMap.CONVEYANCE_MOTOR);
      _feederWheelMotor = new TalonFX(RobotMap.CONVEYANCE_WHEEL);

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
    this.runConveyanceAtPercentPower(Constants.CONVEYANCE_FULL_SPEED_REVERSE);
  }

  public void setConveyanceMaxForward() {
    this.runConveyanceAtPercentPower(Constants.CONVEYANCE_FULL_SPEED);
  }

  public void setConveyanceNormalSpeedForward() {
    this.runConveyanceAtPercentPower(Constants.CONVEYANCE_NORMAL_SPEED);
  }

  public void setConveyanceNormalSpeedReverse() {
    this.runConveyanceAtPercentPower(Constants.CONVEYANCE_NORMAL_REVERSE_SPEED);
  }

  private void runConveyanceAtPercentPower(double magnitude) {
    _conveyanceMotor.set(ControlMode.PercentOutput, magnitude);
  }

  public void stopConveyance() {
    this.runConveyanceAtPercentPower(Constants.CONVEYANCE_STOP);
  }

  public void defaultCommand() {
    this.stopConveyance();
  }

  private void runWheelAtPercentPower(double magnitude) {
    _feederWheelMotor.set(ControlMode.PercentOutput, magnitude);
  }

  public void feederWheelForward() {
    this.runWheelAtPercentPower(Constants.CONVEYANCE_FEEDER_SPEED);
  }

  public void wheelStop() {
    this.runWheelAtPercentPower(Constants.CONVEYANCE_FEEDER_STOP);
  }

  public void feederWheelReverse() {
    this.runWheelAtPercentPower(Constants.CONVEYANCE_REVERSE_FEEDER);
  }

}
