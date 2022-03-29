// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class TurretHomeCommand extends CommandBase {
  /** Creates a new TurretHomeCommand. */
  public TurretHomeCommand() {
    addRequirements(Robot.TURRET_SWIVEL_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.TURRET_SWIVEL_SUBSYSTEM.setPidProfile(Constants.TURRET_DEFAULT_PID);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.TURRET_SWIVEL_SUBSYSTEM.goToAngle(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Robot.TURRET_SWIVEL_SUBSYSTEM.stop
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.TURRET_SWIVEL_SUBSYSTEM.getIsAtTarget();
  }
}
