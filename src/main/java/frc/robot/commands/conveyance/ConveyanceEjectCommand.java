// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.conveyance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/** An example command that uses an example subsystem. */
 public class ConveyanceEjectCommand extends CommandBase {
  public ConveyanceEjectCommand() {
    addRequirements(Robot.CONVEYANCE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("INITIALIZING EJECTION");
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.CONVEYANCE_SUBSYSTEM.setConveyanceEjectCargo();
    Robot.CONVEYANCE_SUBSYSTEM.setIsBallEjecting(true);
    Robot.CONVEYANCE_SUBSYSTEM.setIsConveyanceEjectingWrongColorCargo(true);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();
    
    System.out.println("ENDING EJECTION");
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
