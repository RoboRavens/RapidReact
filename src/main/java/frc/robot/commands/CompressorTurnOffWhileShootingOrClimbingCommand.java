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
    if(Robot.COMPRESSOR_SUBSYSTEM.isClimbing() || Robot.COMPRESSOR_SUBSYSTEM.isShooting()) {
      Robot.COMPRESSOR_SUBSYSTEM.stop();
    }
    else if(Robot.COMPRESSOR_SUBSYSTEM.pressureIsLow()) {
      Robot.COMPRESSOR_SUBSYSTEM.start();
    }
    else {
      Robot.COMPRESSOR_SUBSYSTEM.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
