/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {

  private TalonFX _intakeMotor;
  private Solenoid _intakeExtend;
  private Solenoid _intakeRetract;

  public IntakeSubsystem() {
    this.initialize();
    _intakeMotor = new TalonFX(RobotMap.INTAKE_MOTOR);
    _intakeExtend = new Solenoid(RobotMap.INTAKE_EXTEND_SOLENOID, 0);
    _intakeRetract = new Solenoid( RobotMap.INTAKE_RETRACT_SOLENOID, 0);
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
}