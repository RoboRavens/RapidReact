package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class ConveyanceIntakeCommandGroup extends SequentialCommandGroup {
  
  public ConveyanceIntakeCommandGroup() {
    addCommands(


   // Extends The intake
   new IntakeExtendCommand(
   ),

    // Intakes balls and also brings balls into conveyance stage 2
    new ConveyanceCollectCommand(
     
    ),

    // retracts the intake
    new IntakeRetractCommand(
    )



    );
        
  }
}