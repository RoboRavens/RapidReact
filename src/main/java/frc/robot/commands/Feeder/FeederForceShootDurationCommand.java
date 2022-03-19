// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Feeder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class FeederForceShootDurationCommand extends CommandBase {
  Timer durationTimer = new Timer();
  double duration;

    public FeederForceShootDurationCommand(double durationSeconds) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.FEEDER_SUBSYSTEM, Robot.CONVEYANCE_SUBSYSTEM);
        this.duration = durationSeconds;
    }
 
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      durationTimer.reset();
      durationTimer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Robot.FEEDER_SUBSYSTEM.forceShoot();
        //SmartDashboard.putString("FSOBC Status", "Shooting one ball");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.FEEDER_SUBSYSTEM.feederWheelStop();
        Robot.FEEDER_SUBSYSTEM.conveyanceStop();
        
        //SmartDashboard.putString("FSOBC Status", "DONE SHOOTING");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      boolean isFinished = false;
      
      if (durationTimer.get() >= duration) {
        isFinished = true;
      }

      return isFinished;
    }
}
