// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ClimberRetractCommand extends CommandBase {  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimberRetractCommand() {
    addRequirements(Robot.CLIMBER_SUBSYSTEM);
  }

 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      Robot.CLIMBER_SUBSYSTEM.retractPidLeftSide();
      Robot.CLIMBER_SUBSYSTEM.retractPidRightSide();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      Robot.CLIMBER_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}