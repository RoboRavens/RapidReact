// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.ConveyanceSubsystem;
import frc.robot.subsystems.IntakeExtenderSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


/** An example command that uses an example subsystem. */
public class ConveyanceIndexCommand extends CommandBase {
  private IntakeExtenderSubsystem _conveyanceSensorA;

  public void ConveyanceSubsystem(ConveyanceSubsystem _ConveyanceSubsystem) {
    addRequirements(Robot.FEEDER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.CONVEYANCE_INDEX_COMMAND._conveyanceSensorA()) {
   Robot.FEEDER_SUBSYSTEM.setConveyanceTwoMaxForward();
    } 
  }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
