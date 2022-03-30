// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ClimberRetractCommand extends CommandBase {
  private double _extendedTarget = Constants.CLIMBER_EXTEND_ENCODER_TARGET;
  
  public ClimberRetractCommand() {
    addRequirements(Robot.CLIMBER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double climberExtendError = _extendedTarget - Robot.CLIMBER_SUBSYSTEM.getEncoderPosition();
		double climberSpeed = .5;

		if (climberExtendError >= _extendedTarget * .3) {
			climberSpeed = 1;
		}
		//SmartDashboard.putNumber("climberExtendError", climberExtendError);
		//SmartDashboard.putNumber("climberSpeed", climberSpeed);
		
		Robot.CLIMBER_SUBSYSTEM.retract(climberSpeed * -1);
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
