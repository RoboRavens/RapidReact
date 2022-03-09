// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import frc.controls.AxisCode;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Aims the turret at the target */
public class TurretAimAtTargetCommand extends CommandBase {

  public TurretAimAtTargetCommand() {
    addRequirements(Robot.TURRET_SWIVEL_SUBSYSTEM);
  }

 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.TURRET_SWIVEL_SUBSYSTEM.setShot(Constants.TURRET_DEFAULT_PID);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double angle = Robot.GAMEPAD.getAxis(AxisCode.RIGHTSTICKX);
    Robot.TURRET_SWIVEL_SUBSYSTEM.goToAngle(Robot.LIMELIGHT_SUBSYSTEM.x);
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
