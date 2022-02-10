package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ConveyanceSubsystem;
import frc.robot.subsystems.IntakeExtenderSubsystem;

public class ConveyanceIntakeCommandGroup extends ParallelCommandGroup{
    

    public void ConveyanceEjectCommandGroup(ConveyanceSubsystem conveyanceSubsystem, IntakeExtenderSubsystem intakExtenderSubsystem) {
        addCommands(
            new IntakeExtendCommand() );  //Extends The Intake
            new ConveyanceCollectCommand(); //Collects Cargo
            new IntakeRetractCommand(); //Retracts The Intake
        }
}
