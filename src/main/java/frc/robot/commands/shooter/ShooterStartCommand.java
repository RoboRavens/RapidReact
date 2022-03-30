// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.util.ShooterCalibrationPair;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/** An example command that uses an example subsystem. */
public class ShooterStartCommand extends CommandBase {

  private boolean _manualOverride = false;

  public ShooterStartCommand() {
    addRequirements(Robot.SHOOTER_SUBSYSTEM);
  }

 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!_manualOverride) {
      limelightSetsProfile();
    }
    
    Robot.SHOOTER_SUBSYSTEM.startMotor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void limelightSetsProfile() {
    ShooterCalibrationPair shotToSet = Constants.DISABLED_SHOT_CALIBRATION_PAIR;

    if(Robot.LIMELIGHT_SUBSYSTEM.getRawYOffset() > Constants.MAX_LOW_GOAL_SHOT) { // Close to hub
      shotToSet = Constants.LOW_GOAL_SHOT_CALIBRATION_PAIR;
    } else if(Robot.LIMELIGHT_SUBSYSTEM.getRawYOffset() > Constants.MAX_TARMAC_SHOT) {
      shotToSet = Constants.TARMAC_SHOT_CALIBRATION_PAIR;
    } else if(Robot.LIMELIGHT_SUBSYSTEM.getRawYOffset() > Constants.MAX_AUTO_RADIUS_SHOT) {
      shotToSet = Constants.AUTO_RADIUS_SHOT_CALIBRATION_PAIR;
    } else if(Robot.LIMELIGHT_SUBSYSTEM.getRawYOffset() > Constants.MAX_LAUNCHPAD_SHOT) {
      shotToSet = Constants.LAUNCHPAD_SHOT_CALIBRATION_PAIR;
    }

    if(shotToSet._name != Robot.SHOOTER_SUBSYSTEM.getShot()._name) {
      Robot.SHOOTER_SUBSYSTEM.setShot(shotToSet);
    }
  }
  
  public void enableManual() {
    _manualOverride = true;
  }

  public void disableManual() {
    _manualOverride = false;
  }
}
