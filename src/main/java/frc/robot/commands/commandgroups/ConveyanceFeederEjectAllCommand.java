// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ConveyanceFeederEjectAllCommand extends CommandBase {
  /** Creates a new ConveyancFeederEjectAll. */
  public ConveyanceFeederEjectAllCommand() {
    addRequirements(Robot.CONVEYANCE_SUBSYSTEM);
    addRequirements(Robot.FEEDER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.CONVEYANCE_SUBSYSTEM.setConveyanceEjectCargo();
    Robot.FEEDER_SUBSYSTEM.setFeederConveyanceEject();
    Robot.FEEDER_SUBSYSTEM.feederWheelReverse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();
    Robot.FEEDER_SUBSYSTEM.conveyanceStop();
    Robot.FEEDER_SUBSYSTEM.feederWheelStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
