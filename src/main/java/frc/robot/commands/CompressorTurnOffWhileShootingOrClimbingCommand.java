// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

 import frc.robot.Robot;
import frc.robot.subsystems.ConveyanceSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
 public class CompressorTurnOffWhileShootingOrClimbingCommand extends CommandBase {
  

  public CompressorTurnOffWhileShootingOrClimbingCommand() {
    addRequirements(Robot.COMPRESSOR_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //**refer to slack for further info
    // figure out a way to set the IsClimbing and IsShooting to true when the robot is actually climbing or shooting.
    // probably will have to do this within the Climbing and Shooting Subsystems?
    if(Robot.COMPRESSOR_SUBSYSTEM.IsClimbing() || Robot.COMPRESSOR_SUBSYSTEM.IsShooting()) {
      Robot.COMPRESSOR_SUBSYSTEM.stop();
    }
    // else if the compressor is not full, then run the compressor
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
