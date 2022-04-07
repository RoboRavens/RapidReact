// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/** Aims the turret at the target */
public class TurretAimAtTargetCommand extends CommandBase {

  private double _target = 0;

  public TurretAimAtTargetCommand() {
    addRequirements(Robot.TURRET_SWIVEL_SUBSYSTEM);
  }

  public TurretAimAtTargetCommand(double angle) {
    this();
    _target = angle;
  }

 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.TURRET_SWIVEL_SUBSYSTEM.setPidProfile(Constants.TURRET_DEFAULT_PID);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Robot.LIMELIGHT_SUBSYSTEM.hasTargetSighted()) {
      _target = Robot.TURRET_SWIVEL_SUBSYSTEM.getAngle() - Robot.LIMELIGHT_SUBSYSTEM.getTargetOffsetAngle();
    }
    
    Robot.TURRET_SWIVEL_SUBSYSTEM.goToAngle(_target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void resetTarget() {
    _target = 0;
  }
}
