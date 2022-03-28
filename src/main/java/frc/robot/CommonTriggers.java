package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.controls.AxisCode;
import frc.controls.ButtonCode;

public class CommonTriggers {
    public static Trigger RobotHas2Balls = new Trigger(() -> {
        return Robot.CONVEYANCE_SUBSYSTEM.getConveyanceStagingBeamBreakHasBall() && Robot.FEEDER_SUBSYSTEM.getFeederHasBall();
    });

    public static Trigger RunShooterTrigger = new Trigger(() -> {
        boolean runShooter = false;

        // In either manual or automatic mode, run the shooter if the switch is on.
        if (Robot.OP_PAD.getButtonValue(ButtonCode.SHOOTER_REV)) {
            runShooter = true;
        }

        // In either manual or automatic mode, run the shooter if there's a wrong-colored ball in the feeder.
        if (Robot.FEEDER_SUBSYSTEM.feederHasWrongColorCargo()) {
            runShooter = true;
        }

        // If in auto mode, additionally run the shooter if there are two balls OR the driver is in limelight mode
        if (Robot.OP_PAD.getButtonValue(ButtonCode.SHOOTER_PROFILE_MANUAL_OVERRIDE) == false) {
            if (Robot.getRobotCargoInventory() >= 2) {
                runShooter = true;
            }

            if (Robot.GAMEPAD.getAxisIsPressed(AxisCode.LEFTTRIGGER)) {
                runShooter = true;
            }
        }
        
        return runShooter;
    });

    public static Trigger RunAutoshootingTrigger = new Trigger(() -> {
        boolean robotHasTwoAmmo = false;
        boolean userOverride = false;

        if (Robot.getRobotProperColorInventory() >= 2) {
            robotHasTwoAmmo = true;
        }

        if (Robot.GAMEPAD.getAxisIsPressed(AxisCode.LEFTTRIGGER)) {
            userOverride = true;
        }

        return robotHasTwoAmmo || userOverride;
    });
}
