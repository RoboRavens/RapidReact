package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ConveyanceSubsystem;
import frc.robot.subsystems.FeederSubsystem;

public class ConveyanceEjectCommandGroup extends ParallelCommandGroup{
    public ConveyanceEjectCommandGroup(ConveyanceSubsystem conveyanceSubsystem, FeederSubsystem feederSubsystem) {
        addCommands(
            new ConveyanceEjectCommand(conveyanceSubsystem),
            new FeederEjectCommand()
        );
    }
}
