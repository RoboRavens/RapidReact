/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
see compressor doc at
http://first.wpi.edu/FRC/roborio/beta/docs/java/edu/wpi/first/wpilibj/Compressor.html
*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

public class CompressorSubsystem extends SubsystemBase {

  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  // private boolean _isShooting = false;
  // private boolean _isClimbing = false;
  // private boolean _compressorWasOn = false;

  public CompressorSubsystem() {

  }

  @Override
  public void periodic() {
  }

  // enable/disable the auto function
  public void start() {
    compressor.enableDigital();
  }

  public void stop() {
    compressor.disable();
  }

  // returns electrical current measured in amps
  public double getCurrentAmps() {
    return compressor.getCurrent();
  }

  private Boolean isOn() {
    return compressor.getPressureSwitchValue();
  }

  private Boolean isOff() {
    return !isOn();
  }

  public void Setup() {
    Trigger isClimbing = new Trigger(() -> Robot.CLIMBER_SUBSYSTEM.getIsClimbing());
    Trigger isShooting = new Trigger(() -> Robot.SHOOTER_SUBSYSTEM.getIsShooting());

    isClimbing.or(isShooting)
    .whenActive(this::stop)
    .whenInactive(this::start);
  }
}