// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.util.ShooterCalibration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // CONTROLS
    public static final double JOYSTICK_DEADBAND = .05;
    public static final double DRIVE_MAX_TURN_RADIANS_PER_SECOND = 5;

    //SHOOTER
    public static final int SHOOTER_IDX = 0;
    public static final int SHOOTER_TIMEOUT_MS = 100;

    public static final int TARMAC_RPM = 2000;
    public static final double TARMAC_KF = 0;
    public static final double TARMAC_KP = 0.05;
    public static final double TARMAC_KI = 0;
    public static final double TARMAC_KD = 0;

    public static final int LAUNCHPAD_RPM = 2000;
    public static final double LAUNCHPAD_KF = 0;
    public static final double LAUNCHPAD_KP = 0.05;
    public static final double LAUNCHPAD_KI = 0;
    public static final double LAUNCHPAD_KD = 0;

    public static final ShooterCalibration TARMAC_SHOT = new ShooterCalibration("Tarmac Shot", TARMAC_RPM, TARMAC_KF, TARMAC_KP, TARMAC_KI, TARMAC_KD);
    public static final ShooterCalibration LAUNCHPAD_SHOT = new ShooterCalibration("Launchpad Shot", LAUNCHPAD_RPM, LAUNCHPAD_KF, LAUNCHPAD_KP, LAUNCHPAD_KI, LAUNCHPAD_KD);

    //TURRET SWIVEL

    //CONTROLS
    public static final double AXIS_IS_PRESSED_VALUE = .25;
    public static final double INTAKE_COLLECT_POWER_MAGNITUDE = 0.1;
    public static final double INTAKE_SPIT_POWER_MAGNITUDE = 0.1;


     


    //CONVEYANCE TWO
    //Not sure if these are the correct speeds for the conveyance motor (needs to be calibrated)
    public static final double CONVEYANCE_TWO_FULL_SPEED_REVERSE = -1;
    public static final double CONVEYANCE_TWO_NORMAL_SPEED = 0.75;
    public static final double CONVEYANCE_TWO_NORMAL_REVERSE_SPEED = -0.75;
    public static final double CONVEYANCE_TWO_STOP = 0;
    public static final double CONVEYANCE_TWO_FEEDER_SPEED = 0.75;
	public static final double CONVEYANCE_TWO_FEEDER_STOP = 0;
	public static final double CONVEYANCE_TWO_REVERSE_FEEDER = -.75;
    public static final double CONVEYANCE_TWO_FULL_SPEED = 1;



    //CONVEYANCE ONE
    public static final double CONVEYANCE_ONE_FULL_SPEED_REVERSE = -1;
    public static final double CONVEYANCE_ONE_FULL_SPEED = 1;
    public static final double CONVEYANCE_ONE_NORMAL_SPEED = 0.75;
    public static final double CONVEYANCE_ONE_NORMAL_REVERSE_SPEED = 0.75;
    public static final double CONVEYANCE_ONE_STOP = 0;

    // DRIVETRAIN PATHFINDING
    public static final double TRAJECTORY_CONFIG_MAX_VELOCITY_METERS_PER_SECOND = 2;
    public static final double TRAJECTORY_CONFIG_MAX_ACCELERATION_METERS_PER_SECOND = 2;
    public static final double TRAJECTORY_CONFIG_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
    public static final double TRAJECTORY_CONFIG_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND = Math.PI;

    public static final double SWERVE_CONTROLLER_X_KP = 1;
    public static final double SWERVE_CONTROLLER_Y_KP = 1;
    public static final double SWERVE_CONTROLLER_ANGLE_KP = 4;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints SWERVE_CONTROLLER_ANGULAR_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            TRAJECTORY_CONFIG_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, TRAJECTORY_CONFIG_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND);

}
