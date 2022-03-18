// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ShooterAutoRadiusCommand extends CommandBase {

  public ShooterAutoRadiusCommand() {
    addRequirements(Robot.SHOOTER_SUBSYSTEM);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.SHOOTER_SUBSYSTEM.setShot(Constants.AUTO_RADIUS_SHOT_CALIBRATION_PAIR);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
