package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ConveyanceSubsystem;
import frc.robot.subsystems.IntakeExtenderSubsystem;

public class IntakeCollectCommandGroup extends ParallelCommandGroup{
    public IntakeCollectCommandGroup(ConveyanceSubsystem conveyanceSubsystem, IntakeExtenderSubsystem intakeExtenderSubsystem) {
        addCommands(
            new IntakeExtendCommand(),
            new ConveyanceCollectCommand(),
            new IntakeRetractCommand()
        );
    }
}