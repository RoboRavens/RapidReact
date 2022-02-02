/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {

  private TalonSRX _intakeMotor;
  private Solenoid _intakeExtend;
  private Solenoid _intakeRetract;
  private DigitalInput _conveyanceSensorA;

  public IntakeSubsystem() {
    this.initialize();
    _intakeMotor = new TalonSRX(RobotMap.INTAKE_MOTOR);
    _intakeExtend = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_EXTEND_SOLENOID);
    _intakeRetract = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_RETRACT_SOLENOID);
    _conveyanceSensorA = new DigitalInput(RobotMap.SENSOR_A_CHANNEL);
  }

  public void initialize() {
  }

  public void periodic() {

  }

  public void collect() {
    this.runAtPower(Constants.INTAKE_COLLECT_POWER_MAGNITUDE);
  }

  public void spit() {
    this.runAtPower(Constants.INTAKE_SPIT_POWER_MAGNITUDE * -1);
  }

  public void runAtPower(double magnitude) {
    _intakeMotor.set(ControlMode.PercentOutput, magnitude);
  }

  public void stop() {
    this.runAtPower(0);
  }

  public void extend() {
    _intakeRetract.set(false);
    _intakeExtend.set(true);
  }

  public void retract() {
    _intakeRetract.set(true);
    _intakeExtend.set(false);
  }

  public void stopAndRetract() {
    this.stop();
    this.retract();
  }

  public void defaultCommand() {
    this.stop();
  }

  public boolean getConveyanceSensorAReading() {
    return _conveyanceSensorA.get();
  }
}