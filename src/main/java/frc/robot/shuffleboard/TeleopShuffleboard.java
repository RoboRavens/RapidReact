package frc.robot.shuffleboard;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.controls.ButtonCode;
import frc.robot.Constants;
import frc.robot.Robot;

public class TeleopShuffleboard {
    public TeleopShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Teleop");

        tab.addCamera("USB Camera 0", "USB Camera 0", "usb:/dev/video0")
            .withProperties(Map.of("Show crosshair", false, "Show controls", false))
            .withSize(5, 5)
            .withPosition(0, 0);
        tab.addBoolean("Lime Light Off", () -> Robot.OP_PAD.getButtonValue(ButtonCode.LIMELIGHT_LIGHT_OFF_OVERRIDE))
            .withPosition(6, 0);
        tab.addBoolean("Shooter Rev", () -> Robot.OP_PAD.getButtonValue(ButtonCode.SHOOTER_REV))
            .withPosition(7, 0);
        tab.addBoolean("Climber Override", () -> Robot.CLIMBER_SUBSYSTEM.getOverride())
            .withPosition(8, 0);
        tab.addBoolean("Turret Disabled", () -> Robot.TURRET_SWIVEL_SUBSYSTEM.getTurretEnabled() == false)
            .withPosition(9, 0);
        tab.addBoolean("Launchpad",
            () -> Robot.SHOOTER_SUBSYSTEM.getShot()._name == Constants.LAUNCHPAD_SHOT_CALIBRATION_PAIR._name)
            .withPosition(5, 0);
        tab.addBoolean("Auto Radius",
            () -> Robot.SHOOTER_SUBSYSTEM.getShot()._name  == Constants.AUTO_RADIUS_SHOT_CALIBRATION_PAIR._name)
            .withPosition(5, 1);
        tab.addBoolean("Tarmac",
            () -> Robot.SHOOTER_SUBSYSTEM.getShot()._name  == Constants.TARMAC_SHOT_CALIBRATION_PAIR._name)
            .withPosition(5, 2);
        tab.addBoolean("Low Goal",
            () -> Robot.SHOOTER_SUBSYSTEM.getShot()._name  == Constants.LOW_GOAL_SHOT_CALIBRATION_PAIR._name)
            .withPosition(5, 3);
            tab.addBoolean("Junk Shot",
            () -> Robot.SHOOTER_SUBSYSTEM.getShot()._name  == Constants.JUNK_SHOT_CALIBRATION_PAIR._name)
            .withPosition(5, 4);
        tab.addBoolean("Feeder",
            () -> Robot.FEEDER_SUBSYSTEM.getFeederHasBall())
            .withPosition(6, 2);
        tab.addBoolean("Conveyance",
            () -> Robot.CONVEYANCE_SUBSYSTEM.getConveyanceEntryBeamBreakHasBall() || Robot.CONVEYANCE_SUBSYSTEM.getConveyanceEntryBeamBreakHasBall())
            .withPosition(6, 3);
        tab.addBoolean("Is Aligned", () -> Robot.LIMELIGHT_SUBSYSTEM.isAligned())
            .withSize(3, 3)
            .withPosition(7, 1);
    }

    public void robotInit() {
        var camera = CameraServer.startAutomaticCapture();
        camera.setFPS(Constants.CAMERA_FPS);
        camera.setResolution(Constants.CAMERA_WIDTH, Constants.CAMERA_HEIGHT);
    }

    public void switchToTab() {
        Shuffleboard.selectTab("Teleop");
    }
}
