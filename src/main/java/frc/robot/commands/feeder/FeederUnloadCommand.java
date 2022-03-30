// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feeder;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeederUnloadCommand extends CommandBase {
    Timer _timer = new Timer();

    public FeederUnloadCommand() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.FEEDER_SUBSYSTEM);
    }
 
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        _timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Robot.FEEDER_SUBSYSTEM.forceShoot();
        if (Robot.getRobotCargoInventory() == 0) {
            _timer.start();
        } else {
            _timer.reset();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.FEEDER_SUBSYSTEM.stopFeederAndConveyanceTwo();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // As soon as the feeder no longer has a ball, consider the shot as having been taken.
        return _timer.get() > 0.25;
    }
}
