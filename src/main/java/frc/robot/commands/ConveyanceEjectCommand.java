// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

 import frc.robot.Robot;
import frc.robot.subsystems.ConveyanceSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
 public class ConveyanceEjectCommand extends CommandBase {
  

  public ConveyanceEjectCommand() {
  addRequirements(Robot.CONVEYANCE_SUBSYSTEM);
  }

 // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.CONVEYANCE_SUBSYSTEM.setConveyanceMaxReverse();
   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();
  }
   
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
