// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TurretSeekCommand extends CommandBase {

  public TurretSeekCommand() {
        // Commented out for Livonia Mar 15
//    addRequirements(Robot.TURRET_SWIVEL_SUBSYSTEM, Robot.LIMELIGHT_SUBSYSTEM);
  }

 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      // Commented out for Livonia Mar 15
    //  Robot.TURRET_SWIVEL_SUBSYSTEM.setShot(Constants.TURRET_DEFAULT_PID);
    Robot.LIMELIGHT_SUBSYSTEM.turnLEDOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        // Commented out for Livonia Mar 15
        /*
    if(Robot.TURRET_SWIVEL_SUBSYSTEM.getShot().target == Constants.TURRET_RANGE && Robot.TURRET_SWIVEL_SUBSYSTEM.getIsAtTarget()) {
      Robot.TURRET_SWIVEL_SUBSYSTEM.goToAngle(-1 * Constants.TURRET_RANGE);
  } else if(Robot.TURRET_SWIVEL_SUBSYSTEM.getIsAtTarget()) {
      Robot.TURRET_SWIVEL_SUBSYSTEM.goToAngle(Constants.TURRET_RANGE);
  }
  */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.LIMELIGHT_SUBSYSTEM.turnLEDOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Robot.LIMELIGHT_SUBSYSTEM.getTargetOffsetAngle()) < Constants.DESIRED_TURRET_TARGET_BUFFER && Robot.LIMELIGHT_SUBSYSTEM.hasTargetSighted(); // Replace this with a "target found" thing (turret should auto-disable seek when target found)
  }
}
