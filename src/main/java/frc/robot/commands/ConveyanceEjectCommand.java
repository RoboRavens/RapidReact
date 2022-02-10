// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ConveyanceSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ConveyanceEjectCommand extends CommandBase {
  private final ConveyanceSubsystem _conveyanceSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ConveyanceEjectCommand(ConveyanceSubsystem conveyanceSubsystem) {
    _conveyanceSubsystem = conveyanceSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyanceSubsystem);
  }

 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      _conveyanceSubsystem.setConveyanceMaxReverse();
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
