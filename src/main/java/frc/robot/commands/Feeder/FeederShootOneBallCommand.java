// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Feeder;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeederShootOneBallCommand extends CommandBase {
    public FeederShootOneBallCommand() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.FEEDER_SUBSYSTEM);
    }
 
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Robot.FEEDER_SUBSYSTEM.shoot();
        SmartDashboard.putString("FSOBC Status", "Shooting one ball");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.FEEDER_SUBSYSTEM.feederWheelStop();
        Robot.FEEDER_SUBSYSTEM.conveyanceStop();
        
        SmartDashboard.putString("FSOBC Status", "DONE SHOOTING");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // As soon as the feeder no longer has a ball, consider the shot as having been taken.
        return Robot.FEEDER_SUBSYSTEM.getFeederHasBall() == false;
    }
}
