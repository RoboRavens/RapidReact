package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class ConveyanceIntakeCommandGroup extends SequentialCommandGroup {
  
  public ConveyanceIntakeCommandGroup() {
    addCommands(


   // Drive forward the specified distance
   new IntakeExtendCommand(
    ),

    // Release the hatch
    new ConveyanceCollectCommand(
     
    ),

    // Drive backward the specified distance
    new IntakeRetractCommand(
    )



    );
        
  }
}