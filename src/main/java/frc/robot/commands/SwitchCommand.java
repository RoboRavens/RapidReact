package frc.robot.commands;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.LinkedList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwitchCommand extends CommandBase {
  private final ArrayList<Command> m_commands;
  private final int m_commandToRun;
  private Command m_selectedCommand;
  
  public SwitchCommand(ArrayList<Command> commands, int commandToRun) {
    m_commands = commands;
    m_commandToRun = commandToRun;
  }

  @Override
  public void initialize() {
    // Iterates through the commands passed in the constructor to check for the command to run
    for (int currentCommand = 0; currentCommand <= m_commands.size(); currentCommand++) {
      // If the command to run is equal to the command in this iteration
      if(m_commandToRun == currentCommand + 1) {
        // Set the selected command (to be run) to the command at that index in the array sequence
        m_selectedCommand = m_commands.get(currentCommand);
      }
    }
    if(m_selectedCommand == null) {
      System.out.println("Invalid command selection");
    }
  }

  @Override
  public void execute() {
    m_selectedCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    m_selectedCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return m_selectedCommand.isFinished();
  }

}

