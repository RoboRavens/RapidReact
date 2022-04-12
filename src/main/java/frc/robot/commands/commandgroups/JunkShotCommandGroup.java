// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.feeder.FeederForceShootDurationCommand;
import frc.robot.commands.feeder.FeederShootOneBallCommand;
import frc.robot.commands.turret.TurretMissCommand;

public class JunkShotCommandGroup extends SequentialCommandGroup {

  public JunkShotCommandGroup() {
    addCommands(
    //new ShooterLowGoalCommand(), // Set the fail shot profile

    // NEED TO RE-ADD THIS LINE BUT IT IS DANGEROUS FOR TURRET TESTING
    // new TurretMissCommand().withTimeout(1), // Set the miss shot angle
    // new ShooterStartInstantCommand(), // Rev
    
  new    FeederForceShootDurationCommand(1)
    /*new FeederShootOneBallCommand(), // Shoot 1
    new ShooterStopCommand(), // End
    new TurretAimAtTargetCommand() // End
    COMMENTING TURRET AIM COMMAND OUT AS IT WILL NEVER FINISH - WE PROBABLY JUST WANT TO RESUME DEFAULT COMMAND HERE
    This should be checked/confirmed.
    */
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