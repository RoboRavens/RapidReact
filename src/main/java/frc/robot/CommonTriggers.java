package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.controls.AxisCode;
import frc.controls.ButtonCode;

public class CommonTriggers {
    private static boolean releaseBallTriggerWasTrue = false;

    public static Trigger TurretAimLowGoal = new Trigger(() -> {
        if (Robot.autonomousTriggerOverride) {
            return false;
        }

        return Robot.SHOOTER_SUBSYSTEM.getShot()._name == Constants.LOW_GOAL_SHOT_CALIBRATION_PAIR._name;
    });

    public static Trigger AutosteerDisabledTrigger  = new Trigger(() -> {
        if (Robot.autonomousTriggerOverride == true) {
            return false;
        }

        if (Robot.GAMEPAD.getButtonValue(ButtonCode.LEFTBUMPER)) {
            return true;
        }

        if (Robot.GAMEPAD.getAxisIsPressed(AxisCode.LEFTTRIGGER)) {
            return true;
        }

        return Robot.CONVEYANCE_SUBSYSTEM.getConveyanceStagingBeamBreakHasBall() && Robot.FEEDER_SUBSYSTEM.getFeederHasBall();
    });

    public static Trigger RobotHas2Balls = new Trigger(() -> {
        if (Robot.autonomousTriggerOverride == true) {
            return false;
        }

        return Robot.CONVEYANCE_SUBSYSTEM.getConveyanceStagingBeamBreakHasBall() && Robot.FEEDER_SUBSYSTEM.getFeederHasBall();
    });

    public static Trigger RunShooterTrigger = new Trigger(() -> {
        if (Robot.autonomousTriggerOverride == true) {
            return false;
        }

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
        if (Robot.getRobotCargoInventory() >= 2) {
            runShooter = true;
        }

        if (Robot.GAMEPAD.getAxisIsPressed(AxisCode.LEFTTRIGGER)) {
            runShooter = true;
        }

        return runShooter;
        // return false;
    });

    public static Trigger RunAutoshootingTrigger = new Trigger(() -> {
        if (Robot.autonomousTriggerOverride == true) {
            return false;
        }

        boolean robotHasTwoAmmo = false;
        boolean userOverride = false;

        // if (Robot.getRobotProperColorInventory() >= 2) {
        if (Robot.getRobotCargoInventory() >= 2) {
            robotHasTwoAmmo = true;
        }

        SmartDashboard.putBoolean("LEFTTRIGGERPRESSED", Robot.GAMEPAD.getAxisIsPressed(AxisCode.LEFTTRIGGER));
        if (Robot.GAMEPAD.getAxisIsPressed(AxisCode.LEFTTRIGGER)) {
            userOverride = true;
        }

        return robotHasTwoAmmo || userOverride;
        // return false;
    });

    public static Trigger ReleaseBallTrigger = new Trigger(() -> {
        if (Robot.autonomousTriggerOverride == true) {
            return false;
        }

        boolean limelightIsAligned = false;
        boolean rpmIsCorrect = false;
        boolean inAutoshootingMode = false;
        boolean hasAmmo = false;

        if (Robot.LIMELIGHT_SUBSYSTEM.isAligned()) {
            limelightIsAligned = true;
        }

        if (Robot.SHOOTER_SUBSYSTEM.motorsAreRecovered()) {
            rpmIsCorrect = true;
        }

        if (Robot.SHOOTER_SUBSYSTEM.getAutoShotSelect()) {
            inAutoshootingMode = true;
        }

        // if (Robot.getRobotProperColorInventory() >= 2 || Robot.GAMEPAD.getAxisIsPressed(AxisCode.LEFTTRIGGER)) {
        if (Robot.getRobotCargoInventory() >= 2 || Robot.GAMEPAD.getAxisIsPressed(AxisCode.LEFTTRIGGER)) {
            hasAmmo = true;
        }

        // return limelightIsAligned && rpmIsCorrect && inAutoshootingMode && hasAmmo;
        return false;
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

    public static Trigger RobotHasOneBall = new Trigger(() -> {
        return Robot.getRobotCargoInventory() == 1;
    });

    public static Trigger RobotFinishedShooting = new Trigger(() -> {

        if (ReleaseBallTrigger.get()) {
            releaseBallTriggerWasTrue = true;
        }
        else if (Robot.getRobotCargoInventory() == 0 && releaseBallTriggerWasTrue) {
            releaseBallTriggerWasTrue = false;
            return true;
        }

        return false;
    });

    /*
        If there is one ball in the robot
            rumble once
        If there are two balls in the robot
            rumble twice
        If the ReleaseBallTrigger is active
            rumble continuously
        If the balls have just been shot
            rumble (?)
    */
}
