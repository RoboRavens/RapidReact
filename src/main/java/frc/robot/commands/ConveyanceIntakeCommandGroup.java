package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ConveyanceSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeExtenderSubsystem;

public class ConveyanceIntakeCommandGroup extends ParallelCommandGroup{
    public void ConveyanceEjectCommandGroup(ConveyanceSubsystem conveyanceSubsystem, FeederSubsystem feederSubsystem, IntakeExtenderSubsystem intakeExtenderSubsystem) {
        addCommands(
            new IntakeExtendCommand(),
            new ConveyanceCollectCommand(),
            new IntakeRetractCommand()
        );

     }
    
}