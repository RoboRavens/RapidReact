// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Sets the turret encoder to 0 instantly. */
public class TurretResetEncoderCommand extends CommandBase {

  public TurretResetEncoderCommand() {
    addRequirements(Robot.TURRET_SWIVEL_SUBSYSTEM);
  }

 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      //System.out.println("TurretResetEncoderCommand init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      Robot.TURRET_SWIVEL_SUBSYSTEM.setEncoder(0);
      System.out.println("Reset encoder!");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
