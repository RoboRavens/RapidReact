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
  private boolean _searchTargetIsClockwise = true;

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
    // If the robot sees the goal, immediately set the target and update the output cap.
    if (Robot.LIMELIGHT_SUBSYSTEM.hasTargetSighted()) {
      Robot.TURRET_SWIVEL_SUBSYSTEM.setOutputCap(Constants.TURRET_MOTOR_TRACKING_OUTPUT_CAP);
      _target = Robot.TURRET_SWIVEL_SUBSYSTEM.getAngle() - Robot.LIMELIGHT_SUBSYSTEM.getTargetOffsetAngle();
    }
    else {
      // If we don't see a target, start seeking a target.
      // While seeking, reduce the output cap so the turret oscillates slowly instead of quickly.
      Robot.TURRET_SWIVEL_SUBSYSTEM.setOutputCap(Constants.TURRET_MOTOR_SEARCHING_OUTPUT_CAP);

      // "Is at target" while seeking the goal means the turret is at the end of its range.
      // In this case, switch the target to the far end of the range.
      if (Robot.TURRET_SWIVEL_SUBSYSTEM.getIsAtTarget()) {
        if (_searchTargetIsClockwise) {
          _searchTargetIsClockwise = false;
          _target = Math.min(Constants.TURRET_COUNTER_CLOCKWISE_HARDWARE_LIMIT, Constants.TURRET_RANGE);
        }
        else {
          _searchTargetIsClockwise = true;
          _target = Math.max(Constants.TURRET_CLOCKWISE_HARDWARE_LIMIT, Constants.TURRET_RANGE * -1);
        }
      }
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
