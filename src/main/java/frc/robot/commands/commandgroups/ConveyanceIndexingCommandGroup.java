package frc.robot.commands.commandgroups;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SwitchCommand;
import frc.robot.commands.conveyance.ConveyanceEjectCommand;
import frc.robot.commands.conveyance.ConveyanceStopCommand;
import frc.robot.commands.conveyanceIndexingCommands.IfFeederDoesNotHaveBallCommandGroup;

public class ConveyanceIndexingCommandGroup extends SequentialCommandGroup {

    public class CommandConditionaPair {
        Command _command;
        BooleanSupplier _condition;
        public CommandConditionaPair(Command command, BooleanSupplier condition) {
            _command = command;
            _condition = condition;
        }
    }

    public ConveyanceIndexingCommandGroup() {
        var test = new ArrayList<CommandConditionaPair>();
        test.add(new CommandConditionaPair(new MoveBallToFeeder(), () -> {
            return feederHasBall && conveyanceHasBall;
        }));
        test.add(new CommandConditionaPair(new ConveyanceStopCommand(), () -> {
            return false;
        }));

        addCommands(
            new SwitchCommand(new ArrayList<Command>(List.of(new ConveyanceEjectCommand(), new ConveyanceStopCommand())), 1)
        );
    }
    
}
