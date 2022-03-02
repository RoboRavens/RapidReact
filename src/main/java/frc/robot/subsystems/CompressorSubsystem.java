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

public class CompressorSubsystem extends SubsystemBase {

  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private boolean _isShooting = false;
  private boolean _isClimbing = false;

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

  public Boolean isOn() {
    return compressor.getPressureSwitchValue();
  }

  public Boolean isOff() {
    return !isOn();
  }

  public void setIsShooting(boolean isShooting) {
    _isShooting = isShooting;
  }

  public void setIsClimbing(boolean isClimbing) {
      _isClimbing = isClimbing;
  }

  public boolean IsShooting() {
      return _isShooting;
  }

  public boolean IsClimbing() {
    return _isClimbing;
  }
}