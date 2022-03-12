// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.util.ShooterCalibration;
import frc.util.TurretCalibration;

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
    public static final double DRIVE_MAX_TURN_RADIANS_PER_SECOND = 2;

    //SHOOTER
    public static final int SHOOTER_IDX = 0;
    public static final int SHOOTER_TIMEOUT_MS = 100;
    public static final int SHOOTER_TARGET_ALLOWANCE = 500; //Amount of RPM offset to consider still "on target"

    public static final int SHOOTER_VEL_TO_RPM = 8192 / 600;

    public static final int TARMAC_RPM = 5700;
    public static final double TARMAC_KF = 0;
    public static final double TARMAC_KP = 0.06;
    public static final double TARMAC_KI = 0;
    public static final double TARMAC_KD = 0;

    public static final int LAUNCHPAD_RPM = 4700;
    public static final double LAUNCHPAD_KF = 0;
    public static final double LAUNCHPAD_KP = 0.05;
    public static final double LAUNCHPAD_KI = 0;
    public static final double LAUNCHPAD_KD = 0;

    public static final ShooterCalibration TARMAC_SHOT = new ShooterCalibration("Tarmac Shot", TARMAC_RPM, TARMAC_KF, TARMAC_KP, TARMAC_KI, TARMAC_KD);
    public static final ShooterCalibration LAUNCHPAD_SHOT = new ShooterCalibration("Launchpad Shot", LAUNCHPAD_RPM, LAUNCHPAD_KF, LAUNCHPAD_KP, LAUNCHPAD_KI, LAUNCHPAD_KD);

    //TURRET SWIVEL
    public static final int TURRET_IDX = 0;
    public static final int TURRET_TIMEOUT_MS = 100;
    public static final int TURRET_RANGE = 50; //Degrees of motion in both ways (180 means full movement both ways)
    public static final double TURRET_ENCODER_RATIO = 500; //Divide encoder ticks by this, multiply angles by this (encoder ticks are much less than angles)
    public static final double TURRET_AIM_ALLOWANCE = 2; //Degrees of allowance to say that the turret has "reached" its target
    
    public static final double TURRET_DEFAULT_KF = 0;
    public static final double TURRET_DEFAULT_KP = 0.075;
    public static final double TURRET_DEFAULT_KI = 0;
    public static final double TURRET_DEFAULT_KD = 0;

    public static final double TURRET_FLIP_KF = 0;
    public static final double TURRET_FLIP_KP = 0.1;
    public static final double TURRET_FLIP_KI = 0;
    public static final double TURRET_FLIP_KD = 0;

    public static final TurretCalibration TURRET_DEFAULT_PID = new TurretCalibration("Default", TURRET_DEFAULT_KF, TURRET_DEFAULT_KP, TURRET_DEFAULT_KI, TURRET_DEFAULT_KD);
    public static final TurretCalibration TURRET_FLIP_PID = new TurretCalibration("Flipping", TURRET_FLIP_KF, TURRET_FLIP_KP, TURRET_FLIP_KI, TURRET_FLIP_KD);

    //CONTROLS
    public static final double AXIS_IS_PRESSED_VALUE = .25;
    public static final double INTAKE_COLLECT_POWER_MAGNITUDE = 0.1;
    public static final double INTAKE_SPIT_POWER_MAGNITUDE = 0.1;


     


    //CONVEYANCE TWO
    //Not sure if these are the correct speeds for the conveyance motor (needs to be calibrated)
    public static final double CONVEYANCE_TWO_FULL_SPEED_REVERSE = .25;
    public static final double CONVEYANCE_TWO_NORMAL_SPEED = 0.25;
    public static final double CONVEYANCE_TWO_NORMAL_REVERSE_SPEED = -0.25;
    public static final double CONVEYANCE_TWO_STOP = 0;
    public static final double CONVEYANCE_TWO_FEEDER_SPEED = 0.25;
	public static final double CONVEYANCE_TWO_FEEDER_STOP = 0;
	public static final double CONVEYANCE_TWO_REVERSE_FEEDER = -.25;
    public static final double CONVEYANCE_TWO_FULL_SPEED = -.25;



    //CONVEYANCE ONE
    public static final double CONVEYANCE_ONE_FULL_SPEED_REVERSE = .75;
    public static final double CONVEYANCE_ONE_FULL_SPEED = -75;
    public static final double CONVEYANCE_ONE_NORMAL_SPEED = -.25;
    public static final double CONVEYANCE_ONE_NORMAL_REVERSE_SPEED = -.25;
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

    public static final double DRIVE_ANGLE_DRIFT_CORRECTION_KP = 1;
    public static final double DRIVE_ANGLE_DRIFT_CORRECTION_KI = .01;
    public static final double DRIVE_ANGLE_DRIFT_CORRECTION_KD = 0;
    
    //CLIMBER
    public static final double CLIMBER_HOLD_POSITION_POWER_MAGNITUDE = 0; // .13
	public static final double CLIMBER_EXTEND_POWER_MAGNITUDE = 1;
	public static final double CLIMBER_RETRACT_POWER_MAGNITUDE = -.4;
	public static final double CLIMBER_RETRACT_TO_LATCH_POWER_MAGNITUDE = .2;

    public static final double FEEDER_SAFETY_REVERSE_DURATION = .15;
    
    //LimeLight
    public static final int FLOOR_TO_LIMELIGHT_LENS_HEIGHT = 0;
    public static final int FLOOR_TO_TARGET_CENTER_HEIGHT = 0;
    public static final double CAMERA_ANGLE_OFFSET_FROM_HORIZONTAL = 0;
    public static final double LIMELIGHT_LENS_TO_ROBOT_CENTER_OFFSET_INCHES = 0;
    public static final double MINIMUM_DISTANCE_FROM_LIMELIGHT = 46.0;
	public static final double MAXIMUM_DISTANCE_FROM_LIMELIGHT = 240.0;
    public static final int DESIRED_TURRET_TARGET_BUFFER = 1;
}
