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

    public static Trigger ReleaseBallTrigger = new Trigger(() -> {
        boolean limelightIsALigned = false;
        boolean RPMIsCorrect = false;
        boolean inAutoshootingMode = false;
        boolean haveAmmo = false;

        if (Robot.LIMELIGHT_SUBSYSTEM.isAligned()) {
            limelightIsALigned = true;
        }

        if (Robot.SHOOTER_SUBSYSTEM.motorsAreRecovered()) {
            RPMIsCorrect = true;
        }

        if (Robot.SHOOTER_SUBSYSTEM.getAutoShotSelect()) {
            inAutoshootingMode = true;
        }

        if (Robot.getRobotProperColorInventory() >= 2 || Robot.GAMEPAD.getAxisIsPressed(AxisCode.LEFTTRIGGER)) {
            haveAmmo = true;
        }

        return limelightIsALigned && RPMIsCorrect && inAutoshootingMode && haveAmmo;
    });

    /*
    Release ball when:
        limelight is aligned (method in limelight subsystem)
        RPM is correct (read RPM for both shooter flywheels relative to target; there may be a method for this in the shooter subsystem already, but maybe not)
        we're in autoshooting mode (manual override isn't flipped)
            somwhere in robot.java you'll find a manual override for shooting mode
        we have ammo

        trigger checks all these conditions
        when condition is met (trigger is true) run the feeder wheel's shoot sequence

    */
}
