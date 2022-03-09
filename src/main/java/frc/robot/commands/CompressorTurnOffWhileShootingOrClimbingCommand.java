package frc.robot.commands;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.



// import frc.robot.Robot;

// import edu.wpi.first.wpilibj2.command.CommandBase;

// /** An example command that uses an example subsystem. */
// public class CompressorTurnOffWhileShootingOrClimbingCommand extends CommandBase {
  

//   public CompressorTurnOffWhileShootingOrClimbingCommand() {
//     addRequirements(Robot.COMPRESSOR_SUBSYSTEM);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
      
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if(Robot.CLIMBER_SUBSYSTEM.getIsClimbing() || Robot.SHOOTER_SUBSYSTEM.getIsShooting()) {
//       Robot.COMPRESSOR_SUBSYSTEM.stop();
//     }
//     else {
//       Robot.COMPRESSOR_SUBSYSTEM.start();
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }



public class CompressorTurnOffWhileShootingOrClimbingCommand {

  public class IsClimbingSupplier implements BooleanSupplier {
    @Override
    public boolean getAsBoolean() {
      return Robot.CLIMBER_SUBSYSTEM.getIsClimbing();
    }
  }

  public class IsShootingSupplier implements BooleanSupplier {
    @Override
    public boolean getAsBoolean() {
      return Robot.SHOOTER_SUBSYSTEM.getIsShooting();
    }
  }

  public static void Setup() {
    Trigger isClimbing = new Trigger(new IsClimbingSupplier());
    Trigger isShooting = new Trigger(new IsShootingSupplier());

    isClimbing.or(isShooting)
    .whenActive(Robot.COMPRESSOR_SUBSYSTEM::stop)
    .whenInactive(Robot.COMPRESSOR_SUBSYSTEM::start);
  }
}