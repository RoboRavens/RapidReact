package frc.robot.commands;

import java.util.ArrayList;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.commandgroups.ConveyanceIndexingCommandGroup.CommandConditionalPair;

public class SwitchCommand extends CommandBase {
  private final ArrayList<CommandConditionalPair> _commandConditionalPairs;
  private Command _selectedCommand;
  
  public SwitchCommand(ArrayList<CommandConditionalPair> commandConditionalPairs) {
    _commandConditionalPairs = commandConditionalPairs;
  }

  @Override
  public void initialize() {
    for (CommandConditionalPair commandConditionalPair : _commandConditionalPairs) {
      if (commandConditionalPair.getCommandCondition().getAsBoolean()) {
        _selectedCommand = commandConditionalPair.getCommand();
        _selectedCommand.initialize();
        break;
      }
    }
    if (_selectedCommand == null) {
      end(true);
    }
  }

  @Override
  public void execute() {
    _selectedCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    if (_selectedCommand != null) {
      _selectedCommand.end(interrupted);
    }
  }

  @Override
  public boolean isFinished() {
    return _selectedCommand.isFinished();
  }

}

