// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Feeder;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class FeederSafetyReverseCommand extends CommandBase {
    double _durationInSeconds;
    Timer _timer;
  
    public FeederSafetyReverseCommand(double durationInSeconds) {
      addRequirements(Robot.FEEDER_SUBSYSTEM);
      _durationInSeconds = durationInSeconds;
      _timer = new Timer();
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      // System.out.println("ConveyanceReverseForDurationCommand initialized");
      _timer.reset();
      _timer.start();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      //Robot.FEEDER_SUBSYSTEM.setConveyanceMaxReverse();
      Robot.FEEDER_SUBSYSTEM.feederWheelReverse();
      // System.out.println("REVERSING_CONVEYOR!!!");
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      Robot.FEEDER_SUBSYSTEM.feederWheelStop();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      if (_timer.get() > _durationInSeconds) {
        return true;
      }
      return false;
    }
}
