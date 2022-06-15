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
        addRequirements(Robot.CONVEYANCE_SUBSYSTEM);
        
        BooleanSupplier entranceBeamBreakHasBall = () -> Robot.CONVEYANCE_SUBSYSTEM.getConveyanceEntryBeamBreakHasBall();
        BooleanSupplier stagingBeamBreakHasBall = () -> Robot.CONVEYANCE_SUBSYSTEM.getConveyanceStagingBeamBreakHasBall();
        BooleanSupplier feederHasBall = () -> Robot.FEEDER_SUBSYSTEM.getFeederHasBall();
        BooleanSupplier ballIsEjecting = () -> Robot.CONVEYANCE_SUBSYSTEM.getIsBallEjecting();
        BooleanSupplier conveyanceEjectingWrongColorCargo = () -> Robot.CONVEYANCE_SUBSYSTEM.getIsConveyanceEjectingWrongColorCargo();
        BooleanSupplier conveyanceEjectingThirdBall = () -> Robot.CONVEYANCE_SUBSYSTEM.getConveyanceEjectingThirdBall();
        BooleanSupplier stagingEjectionPassThroughIsOccurring = () -> Robot.CONVEYANCE_SUBSYSTEM.getStagingEjectionPassThroughIsOccurring();

        ArrayList<CommandConditionalPair> indexingCommandConditionalPairs = new ArrayList<CommandConditionalPair>();

        indexingCommandConditionalPairs.add(new CommandConditionalPair(new SetIsIndexingFromEntranceToStagingCommand(false), () -> {
            return feederHasBall.getAsBoolean() &&
            stagingBeamBreakHasBall.getAsBoolean() &&
            ballIsEjecting.getAsBoolean() == false;
        }));

        indexingCommandConditionalPairs.add(new CommandConditionalPair(new ConveyanceStopCommand(), () -> {
            return feederHasBall.getAsBoolean() &&
            stagingBeamBreakHasBall.getAsBoolean() &&
            ballIsEjecting.getAsBoolean() == false &&
            Robot.CONVEYANCE_SUBSYSTEM.conveyanceHasProperColorCargo() &&
            entranceBeamBreakHasBall.getAsBoolean() == false;
        }));

        indexingCommandConditionalPairs.add(new CommandConditionalPair(new ConveyanceEjectThirdBallCommand(), () -> {
          return feederHasBall.getAsBoolean() &&
          stagingBeamBreakHasBall.getAsBoolean() &&
          ballIsEjecting.getAsBoolean() == false &&
          Robot.CONVEYANCE_SUBSYSTEM.conveyanceHasProperColorCargo() &&
          entranceBeamBreakHasBall.getAsBoolean();
        }));

        indexingCommandConditionalPairs.add(new CommandConditionalPair(new ConveyanceEjectCommand(), () -> {
            return feederHasBall.getAsBoolean() &&
            stagingBeamBreakHasBall.getAsBoolean() &&
            ballIsEjecting.getAsBoolean() == false &&
            Robot.CONVEYANCE_SUBSYSTEM.conveyanceHasWrongColorCargo();
        }));

        indexingCommandConditionalPairs.add(new CommandConditionalPair(new ConveyanceIndexCargoForward(true), () -> {
            return feederHasBall.getAsBoolean() &&
            stagingBeamBreakHasBall.getAsBoolean() == false &&
            ballIsEjecting.getAsBoolean() == false &&
            (entranceBeamBreakHasBall.getAsBoolean() || Robot.CONVEYANCE_SUBSYSTEM.getIsIndexingFromEntranceToStaging());
        }));

        indexingCommandConditionalPairs.add(new CommandConditionalPair(new ConveyanceStopCommand(), () -> {
            return feederHasBall.getAsBoolean() && 
            stagingBeamBreakHasBall.getAsBoolean() == false && 
            ballIsEjecting.getAsBoolean() == false &&
            entranceBeamBreakHasBall.getAsBoolean() == false &&
            Robot.CONVEYANCE_SUBSYSTEM.getIsIndexingFromEntranceToStaging() == false;
        }));

        indexingCommandConditionalPairs.add(new CommandConditionalPair(new ConveyanceEjectCommand(), () -> {
            return feederHasBall.getAsBoolean() &&
            ballIsEjecting.getAsBoolean() &&
            conveyanceEjectingWrongColorCargo.getAsBoolean();
        }));

        indexingCommandConditionalPairs.add(new CommandConditionalPair(new ConveyanceNotIndexingCommand(), () -> {
            return feederHasBall.getAsBoolean() &&
            ballIsEjecting.getAsBoolean() &&
            conveyanceEjectingWrongColorCargo.getAsBoolean() &&
            entranceBeamBreakHasBall.getAsBoolean() == false &&
            stagingEjectionPassThroughIsOccurring.getAsBoolean();
        }));

        indexingCommandConditionalPairs.add(new CommandConditionalPair(new ConveyanceNotEjectingCommand(), () -> {
            return feederHasBall.getAsBoolean() &&
            ballIsEjecting.getAsBoolean() &&
            conveyanceEjectingWrongColorCargo.getAsBoolean() == false &&
            conveyanceEjectingThirdBall.getAsBoolean() &&
            entranceBeamBreakHasBall.getAsBoolean() == false;
        }));

        indexingCommandConditionalPairs.add(new CommandConditionalPair(new ConveyanceStopCommand(), () -> {
            return feederHasBall.getAsBoolean() == false &&
            Robot.getRobotCargoInventory() == 0;
        }));

        indexingCommandConditionalPairs.add(new CommandConditionalPair(new ConveyanceIndexCargoForward(false), () -> {
            return feederHasBall.getAsBoolean() == false &&
            Robot.getRobotCargoInventory() != 0 &&
            stagingBeamBreakHasBall.getAsBoolean();
        }));

        indexingCommandConditionalPairs.add(new CommandConditionalPair(new ConveyanceIndexCargoForward(true), () -> {
            return feederHasBall.getAsBoolean() == false &&
            Robot.getRobotCargoInventory() != 0 &&
            stagingBeamBreakHasBall.getAsBoolean() == false &&
            (Robot.CONVEYANCE_SUBSYSTEM.getIsIndexingFromEntranceToStaging() || entranceBeamBreakHasBall.getAsBoolean());
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

    public class ConveyanceEjectCommand extends CommandBase {
        public ConveyanceEjectCommand() {
          addRequirements(Robot.CONVEYANCE_SUBSYSTEM);
        }
      
        @Override
        public void initialize() {}
        
        @Override
        public void execute() {
          Robot.CONVEYANCE_SUBSYSTEM.setConveyanceEjectCargo();
          Robot.CONVEYANCE_SUBSYSTEM.setIsBallEjecting(true);
          Robot.CONVEYANCE_SUBSYSTEM.setIsConveyanceEjectingWrongColorCargo(true);
        }
        
        @Override
        public void end(boolean interrupted) {
          Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();
        }
        
        @Override
        public boolean isFinished() {
          return true;
        }
      }

    public class ConveyanceIndexCargoForward extends CommandBase {
        private boolean _indexingFromEntranceToStaging;

        public ConveyanceIndexCargoForward(boolean indexingFromEntranceToStaging) {
            _indexingFromEntranceToStaging = indexingFromEntranceToStaging;
            addRequirements(Robot.CONVEYANCE_SUBSYSTEM);
        }
      
        @Override
        public void initialize() {}
        
        @Override
        public void execute() {
            Robot.CONVEYANCE_SUBSYSTEM.setConveyanceIndexCargoForward();
            Robot.CONVEYANCE_SUBSYSTEM.setIsIndexingFromEntranceToStaging(_indexingFromEntranceToStaging);
        }
        
        @Override
        public void end(boolean interrupted) {}
        
        @Override
        public boolean isFinished() {
          return true;
        }
    } 

    public class ConveyanceNotIndexingCommand extends CommandBase {

        public ConveyanceNotIndexingCommand() {
            addRequirements(Robot.CONVEYANCE_SUBSYSTEM);
        }
      
        @Override
        public void initialize() {}
        
        @Override
        public void execute() {
            Robot.CONVEYANCE_SUBSYSTEM.setStagingEjectionPassThroughIsOccurring(false);
            Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();
            Robot.CONVEYANCE_SUBSYSTEM.setIsBallEjecting(false);
            Robot.CONVEYANCE_SUBSYSTEM.setIsConveyanceEjectingWrongColorCargo(false);
            Robot.CONVEYANCE_SUBSYSTEM.setIsIndexingFromEntranceToStaging(false);
        }
        
        @Override
        public void end(boolean interrupted) {}
        
        @Override
        public boolean isFinished() {
          return true;
        }
    } 

    public class ConveyanceNotEjectingCommand extends CommandBase {

        public ConveyanceNotEjectingCommand() {
            addRequirements(Robot.CONVEYANCE_SUBSYSTEM);
        }
      
        @Override
        public void initialize() {}
        
        @Override
        public void execute() {
            Robot.CONVEYANCE_SUBSYSTEM.stopConveyanceOne();
            Robot.CONVEYANCE_SUBSYSTEM.setIsBallEjecting(false);
            Robot.CONVEYANCE_SUBSYSTEM.setConveyanceEjectingThirdBall(false);
        }
        
        @Override
        public void end(boolean interrupted) {}
        
        @Override
        public boolean isFinished() {
          return true;
        }
    } 
    
}
