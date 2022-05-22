package frc.robot.commands.commandgroups;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.SwitchCommand;

public class ConveyanceIndexingCommandGroup extends SequentialCommandGroup {

    public class CommandConditionalPair {
        private Command _command;
        private BooleanSupplier _condition;

        public CommandConditionalPair(Command command, BooleanSupplier condition) {
            _command = command;
            _condition = condition;
        }

        public Command getCommand() {
          return _command;
        }

        public BooleanSupplier getCommandCondition() {
          return _condition;
        }
    }

    public ConveyanceIndexingCommandGroup() {
        boolean entranceBeamBreakHasBall = Robot.CONVEYANCE_SUBSYSTEM.getConveyanceEntryBeamBreakHasBall();
        boolean stagingBeamBreakHasBall = Robot.CONVEYANCE_SUBSYSTEM.getConveyanceStagingBeamBreakHasBall();
        boolean feederHasBall = Robot.FEEDER_SUBSYSTEM.getFeederHasBall();
        boolean ballIsEjecting = Robot.CONVEYANCE_SUBSYSTEM.getIsBallEjecting();
        boolean conveyanceEjectingWrongColorCargo = Robot.CONVEYANCE_SUBSYSTEM.getIsConveyanceEjectingWrongColorCargo();
        boolean conveyanceEjectingThirdBall = Robot.CONVEYANCE_SUBSYSTEM.getConveyanceEjectingThirdBall();
        boolean stagingEjectionPassThroughIsOccurring = Robot.CONVEYANCE_SUBSYSTEM.getStagingEjectionPassThroughIsOccurring();

        ArrayList<CommandConditionalPair> indexingCommandConditionalPairs = new ArrayList<CommandConditionalPair>();

        indexingCommandConditionalPairs.add(new CommandConditionalPair(new SetIsIndexingFromEntranceToStagingCommand(false), () -> {
            return feederHasBall &&
            stagingBeamBreakHasBall &&
            ballIsEjecting == false;
        }));

        indexingCommandConditionalPairs.add(new CommandConditionalPair(new ConveyanceStopCommand(), () -> {
            return feederHasBall &&
            stagingBeamBreakHasBall &&
            ballIsEjecting == false &&
            Robot.CONVEYANCE_SUBSYSTEM.conveyanceHasProperColorCargo() &&
            entranceBeamBreakHasBall == false;
        }));

        indexingCommandConditionalPairs.add(new CommandConditionalPair(new ConveyanceEjectThirdBallCommand(), () -> {
          return feederHasBall &&
          stagingBeamBreakHasBall &&
          ballIsEjecting == false &&
          Robot.CONVEYANCE_SUBSYSTEM.conveyanceHasProperColorCargo() &&
          entranceBeamBreakHasBall;
        }));

        addCommands(
            new SwitchCommand(indexingCommandConditionalPairs)
        );
    }

    // The following classes are the various commands that only run within the conveyanceIndexingCommandGroup
    
    public class SetIsIndexingFromEntranceToStagingCommand extends CommandBase {
        private boolean _isIndexingFromEntranceToStaging = false;

        public SetIsIndexingFromEntranceToStagingCommand(boolean isIndexingFromEntranceToStaging) {
            _isIndexingFromEntranceToStaging = isIndexingFromEntranceToStaging;
        }
        @Override
        public void initialize() {}
        
        @Override
        public void execute() {
          Robot.CONVEYANCE_SUBSYSTEM.setIsIndexingFromEntranceToStaging(_isIndexingFromEntranceToStaging);
        }
        
        @Override
        public void end(boolean interrupted) {}
        
        @Override
        public boolean isFinished() {
          return true;
        }
    }

    public class ConveyanceStopCommand extends CommandBase {
  
        public ConveyanceStopCommand() {
          addRequirements(Robot.CONVEYANCE_SUBSYSTEM);
        }
      
        @Override
        public void initialize() {}
        
        @Override
        public void execute() {
          Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();
        }
        
        @Override
        public void end(boolean interrupted) {}
        
        @Override
        public boolean isFinished() {
          return true;
        }

    }

    public class ConveyanceEjectThirdBallCommand extends CommandBase {
  
      public ConveyanceEjectThirdBallCommand() {
        addRequirements(Robot.CONVEYANCE_SUBSYSTEM);
      }
    
      @Override
      public void initialize() {}

      @Override
      public void execute() {
        Robot.CONVEYANCE_SUBSYSTEM.setConveyanceEjectCargo();
        Robot.CONVEYANCE_SUBSYSTEM.setIsBallEjecting(true);
        Robot.CONVEYANCE_SUBSYSTEM.setConveyanceEjectingThirdBall(true);
        Robot.CONVEYANCE_SUBSYSTEM.setStagingEjectionPassThroughIsOccurring(false);
      }
      
      @Override
      public void end(boolean interrupted) {}
      
      @Override
      public boolean isFinished() {
        return true;
      }
    }
    
}
