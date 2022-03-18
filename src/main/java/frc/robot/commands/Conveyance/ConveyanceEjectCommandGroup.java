package frc.robot.commands.Conveyance;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Feeder.FeederEjectCommand;
import frc.robot.subsystems.ConveyanceSubsystem;
import frc.robot.subsystems.FeederSubsystem;

public class ConveyanceEjectCommandGroup extends ParallelCommandGroup{
    public ConveyanceEjectCommandGroup(ConveyanceSubsystem conveyanceSubsystem, FeederSubsystem feederSubsystem) {
        addCommands(
            new ConveyanceEjectCommand(),
            new FeederEjectCommand()
        );
    }
}