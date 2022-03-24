// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroup;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Feeder.FeederShootOneBallCommand;
import frc.robot.commands.shooter.ShooterLowGoalCommand;
import frc.robot.commands.shooter.ShooterStartInstantCommand;
import frc.robot.commands.shooter.ShooterStopCommand;
import frc.robot.commands.turret.TurretAimAtTargetCommand;
import frc.robot.commands.turret.TurretMissCommand;

public class JunkShotCommandGroup extends SequentialCommandGroup {
  
  public JunkShotCommandGroup() {
    addCommands(
    new ShooterLowGoalCommand(), // Set the fail shot profile
    new TurretMissCommand(), // Set the miss shot angle
    new ShooterStartInstantCommand(), // Rev
    new FeederShootOneBallCommand(), // Shoot 1
    new ShooterStopCommand(), // End
    new TurretAimAtTargetCommand() // End
    );
  }

  /**
     * Deadline: 
     *  - feeder wheels feeding one shot (wait for a moment to let shooter rev!! maybe have it tied to shooter revving?)
     * Commands:
     *  - set failure shot profile
     *  - set turret target to failure aiming
     *  - rev shooter
     * 
     * Things to keep from stopping before the group ends:
     * - Shooter rev
     * - Turret miss
     */
}
