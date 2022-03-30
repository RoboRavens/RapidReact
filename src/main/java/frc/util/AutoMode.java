package frc.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoMode {
    private String _name;
    private Command _command;

    public AutoMode(String name, Command command) {
        _name = name;
        _command = command;
    }

    public String getAutoName() {
        return _name;
    }

    public Command getAutoCommand() {
        return _command;
    }

    public void setDefaultOption(SendableChooser<AutoMode> chooser) {
        chooser.setDefaultOption(_name, this);
    }

    public void addOption(SendableChooser<AutoMode> chooser) {
        chooser.addOption(_name, this);
    }
}
