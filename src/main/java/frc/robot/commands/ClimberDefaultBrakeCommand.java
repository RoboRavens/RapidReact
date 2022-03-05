// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

 import frc.robot.Robot;
import frc.robot.subsystems.ConveyanceSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
 public class ClimberDefaultBrakeCommand extends CommandBase {

  public ClimberDefaultBrakeCommand() {
    addRequirements(Robot.CLIMBER_SUBSYSTEM);
  }

 // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.CLIMBER_SUBSYSTEM.brakeClimbers();
   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.CLIMBER_SUBSYSTEM.brakeClimbers();
  }
   
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
