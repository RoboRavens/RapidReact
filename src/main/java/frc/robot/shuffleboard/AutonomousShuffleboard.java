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
    private SendableChooser<Integer> _autoChooser = new SendableChooser<>();
    private final AutoMode[] _autos = new AutoMode[7];

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
        _autos[0] = TwoBallAutoCommand.getHangarAutoMode();
        _autos[1] = TwoBallAutoCommand.getWallAutoMode();
        _autos[2] = ThreeBallTarmacAutoCommand.getAutoMode();
        _autos[3] = FiveBallHps.getAutoMode();
        _autos[4] = TwoBallAutoCommand.getHangarPlusOtherBallsHangarAutoMode();
        _autos[5] = TwoBallAutoCommand.getHangarPlusOtherBallsByGoalAutoMode();
        _autos[6] = new AutoMode("Do Nothing", new InstantCommand());

        _autoChooser.setDefaultOption(_autos[0].getAutoName(), 0);
        for(int i = 1; i < _autos.length; i++){
            _autoChooser.addOption(_autos[i].getAutoName(), i);
        }
    }

    public void switchToTab() {
        Shuffleboard.selectTab("Autonomous");
    }

    public AutoMode getAuto() {
        var autoModeInt = _autoChooser.getSelected();
        if (autoModeInt == null) {
            return _autos[0];
        }

        return _autos[autoModeInt];
    }
}
