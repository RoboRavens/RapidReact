package frc.robot.shuffleboard;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.auto.FiveBallHps;
import frc.robot.commands.auto.ThreeBallTarmacAutoCommand;
import frc.robot.commands.auto.TwoBallAutoCommand;
import frc.util.AutoMode;

public class AutonomousShuffleboard {
    private SendableChooser<AutoMode> _autoChooser = new SendableChooser<>();
    public static final AutoMode TWO_BALL_HANGAR_AUTO = TwoBallAutoCommand.getHangarAutoMode();

    private final NetworkTableEntry _chosenAuto;
    
    public AutonomousShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
        tab.add("Atonomous Mode Select", _autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2,1);
        _chosenAuto = tab.add("Verify Autonomous Mode", "not chosen").withPosition(0, 1).withSize(2, 1).getEntry();
    }

    public void robotPeriodic() {
        _chosenAuto.setString(this.getAuto().getAutoName());
    }

    public void robotInit() {
        AutoMode twoBallWall = TwoBallAutoCommand.getWallAutoMode();
        AutoMode threeBallTarmac = ThreeBallTarmacAutoCommand.getAutoMode();
        AutoMode fiveBallHps = FiveBallHps.getAutoMode();
        AutoMode twoBallHangarPlusHangar = TwoBallAutoCommand.getHangarPlusOtherBallsHangarAutoMode();
        AutoMode twoBallHangarPlusGoal = TwoBallAutoCommand.getHangarPlusOtherBallsByGoalAutoMode();
        AutoMode doNothing = new AutoMode("Do Nothing", new InstantCommand());
    
        TWO_BALL_HANGAR_AUTO.setDefaultOption(_autoChooser);
        twoBallWall.addOption(_autoChooser);
        threeBallTarmac.addOption(_autoChooser);
        fiveBallHps.addOption(_autoChooser);
        twoBallHangarPlusHangar.addOption(_autoChooser);
        twoBallHangarPlusGoal.addOption(_autoChooser);
        doNothing.addOption(_autoChooser);
    }

    public void switchToTab() {
        Shuffleboard.selectTab("Autonomous");
    }

    public AutoMode getAuto() {
        var autoMode = _autoChooser.getSelected();
        if (autoMode == null) {
            return TWO_BALL_HANGAR_AUTO;
        }

        return autoMode;
    }
}
