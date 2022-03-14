// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.util.ShooterCalibration;
import frc.util.ShooterCalibrationPair;
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
    // TALONFX
    public static final double TALONFX_TICKS_PER_REVOLUTION = 2048;
    public static final double TALON_TPS_TO_RPM = 600;
    public static final double TALON_VELOCITY_TO_RPM = TALON_TPS_TO_RPM / TALONFX_TICKS_PER_REVOLUTION;
    public static final double TALON_RPM_TO_VELOCITY = 1 / TALON_VELOCITY_TO_RPM;

    // CONTROLS
    public static final double JOYSTICK_DEADBAND = .05;
    public static final double DRIVE_MAX_TURN_RADIANS_PER_SECOND = 2;

    //SHOOTER
    public static final int SHOOTER_IDX = 0;
    public static final int SHOOTER_TIMEOUT_MS = 100;
    public static final int SHOOTER_TARGET_ALLOWANCE = 50; //Amount of RPM offset to consider still "on target"

    public static final double BACKSPIN_GEAR_RATIO = 16.0 / 36.0;
    public static final double TOPSPIN_GEAR_RATIO = 16.0 / 24.0;

    //public static final int SHOOTER_BACKSPIN_VEL_TO_RPM = 8192 / 600;
    //public static final int SHOOTER_TOPSPIN_VEL_TO_RPM = SHOOTER_BACKSPIN_VEL_TO_RPM * 2/1; // Big Wheel:Small Wheel

    // All shots need to be tuned.
    public static final int LOW_GOAL_BACKSPIN_RPM = 1000;
    public static final double LOW_GOAL_BACKSPIN_KF = 0.1043;  
 //   public static final double LOW_GOAL_BACKSPIN_KF = 0.0;
    public static final double LOW_GOAL_BACKSPIN_KP = 0;//0.17;
    public static final double LOW_GOAL_BACKSPIN_KI = 0;
    public static final double LOW_GOAL_BACKSPIN_KD = 0;

    public static final int LOW_GOAL_TOPSPIN_RPM = 1000;
    public static final double LOW_GOAL_TOPSPIN_KF = 0.106;
//    public static final double LOW_GOAL_TOPSPIN_KF = 0.0;
    public static final double LOW_GOAL_TOPSPIN_KP = 0;//0.17;
    public static final double LOW_GOAL_TOPSPIN_KI = 0;
    public static final double LOW_GOAL_TOPSPIN_KD = 0;

    public static final int TARMAC_BACKSPIN_RPM = 1825;
    public static final double TARMAC_BACKSPIN_KF = 0.1033;
//    public static final double TARMAC_BACKSPIN_KF = 0.0;
    public static final double TARMAC_BACKSPIN_KP = 0;//0.21;
    public static final double TARMAC_BACKSPIN_KI = 0;
    public static final double TARMAC_BACKSPIN_KD = 0;

    public static final int TARMAC_TOPSPIN_RPM = 1825;
    public static final double TARMAC_TOPSPIN_KF = 0.1065;
//    public static final double TARMAC_TOPSPIN_KF = 0.0;
    public static final double TARMAC_TOPSPIN_KP = 0;//0.21;
    public static final double TARMAC_TOPSPIN_KI = 0;
    public static final double TARMAC_TOPSPIN_KD = 0;

    public static final int AUTO_RADIUS_BACKSPIN_RPM = 1925;
//    public static final double AUTO_RADIUS_BACKSPIN_KF = 0.0;
    public static final double AUTO_RADIUS_BACKSPIN_KF = 0.1038;
    public static final double AUTO_RADIUS_BACKSPIN_KP = 0;//0.21;
    public static final double AUTO_RADIUS_BACKSPIN_KI = 0;
    public static final double AUTO_RADIUS_BACKSPIN_KD = 0;

    public static final int AUTO_RADIUS_TOPSPIN_RPM = 1925;
    public static final double AUTO_RADIUS_TOPSPIN_KF = 0.1068;
    //public static final double AUTO_RADIUS_TOPSPIN_KF = 0.0;
    public static final double AUTO_RADIUS_TOPSPIN_KP = 0;//0.21;
    public static final double AUTO_RADIUS_TOPSPIN_KI = 0;
    public static final double AUTO_RADIUS_TOPSPIN_KD = 0;

    public static final int LAUNCHPAD_BACKSPIN_RPM = 2000;
   // public static final double LAUNCHPAD_BACKSPIN_KF = 0.0;
    public static final double LAUNCHPAD_BACKSPIN_KF = 0.1039;
    public static final double LAUNCHPAD_BACKSPIN_KP = 0;//0.22;
    public static final double LAUNCHPAD_BACKSPIN_KI = 0;
    public static final double LAUNCHPAD_BACKSPIN_KD = 0;

    public static final int LAUNCHPAD_TOPSPIN_RPM = 2000;
    public static final double LAUNCHPAD_TOPSPIN_KF = 0.1068;
    //public static final double LAUNCHPAD_TOPSPIN_KF = 0.0;
    public static final double LAUNCHPAD_TOPSPIN_KP = 0;//0.22;
    public static final double LAUNCHPAD_TOPSPIN_KI = 0;
    public static final double LAUNCHPAD_TOPSPIN_KD = 0;

    public static final ShooterCalibration LOW_GOAL_SHOT_BACKSPIN_CALIBRATION = new ShooterCalibration("Low Goal Shot", LOW_GOAL_BACKSPIN_RPM, LOW_GOAL_BACKSPIN_KF, LOW_GOAL_BACKSPIN_KP, LOW_GOAL_BACKSPIN_KI, LOW_GOAL_BACKSPIN_KD);
    public static final ShooterCalibration LOW_GOAL_SHOT_TOPSPIN_CALIBRATION = new ShooterCalibration("Low Goal Alt Shot", LOW_GOAL_TOPSPIN_RPM, LOW_GOAL_TOPSPIN_KF, LOW_GOAL_TOPSPIN_KP, LOW_GOAL_TOPSPIN_KI, LOW_GOAL_TOPSPIN_KD);
    
    public static final ShooterCalibration TARMAC_SHOT_BACKSPIN_CALIBRATION = new ShooterCalibration("Tarmac Shot", TARMAC_BACKSPIN_RPM, TARMAC_BACKSPIN_KF, TARMAC_BACKSPIN_KP, TARMAC_BACKSPIN_KI, TARMAC_BACKSPIN_KD);
    public static final ShooterCalibration TARMAC_SHOT_TOPSPIN_CALIBRATION = new ShooterCalibration("Tarmac Alt Shot", TARMAC_TOPSPIN_RPM, TARMAC_TOPSPIN_KF, TARMAC_TOPSPIN_KP, TARMAC_TOPSPIN_KI, TARMAC_TOPSPIN_KD);

    public static final ShooterCalibration AUTO_RADIUS_SHOT_BACKSPIN_CALIBRATION = new ShooterCalibration("Auto Radius Shot", AUTO_RADIUS_BACKSPIN_RPM, AUTO_RADIUS_BACKSPIN_KF, AUTO_RADIUS_BACKSPIN_KP, AUTO_RADIUS_BACKSPIN_KI, AUTO_RADIUS_BACKSPIN_KD);
    public static final ShooterCalibration AUTO_RADIUS_SHOT_TOPSPIN_CALIBRATION = new ShooterCalibration("Auto Radius Alt Shot", AUTO_RADIUS_TOPSPIN_RPM, AUTO_RADIUS_TOPSPIN_KF, AUTO_RADIUS_TOPSPIN_KP, AUTO_RADIUS_TOPSPIN_KI, AUTO_RADIUS_TOPSPIN_KD);

    public static final ShooterCalibration LAUNCHPAD_SHOT_BACKSPIN_CALIBRATION = new ShooterCalibration("Launchpad Shot", LAUNCHPAD_BACKSPIN_RPM, LAUNCHPAD_BACKSPIN_KF, LAUNCHPAD_BACKSPIN_KP, LAUNCHPAD_BACKSPIN_KI, LAUNCHPAD_BACKSPIN_KD);
    public static final ShooterCalibration LAUNCHPAD_SHOT_TOPSPIN_CALIBRATION = new ShooterCalibration("Launchpad Alt Shot", LAUNCHPAD_TOPSPIN_RPM, LAUNCHPAD_TOPSPIN_KF, LAUNCHPAD_TOPSPIN_KP, LAUNCHPAD_TOPSPIN_KI, LAUNCHPAD_TOPSPIN_KD);

    public static final ShooterCalibrationPair LOW_GOAL_SHOT_CALIBRATION_PAIR = new ShooterCalibrationPair("Low Goal Shot", LOW_GOAL_SHOT_BACKSPIN_CALIBRATION, LOW_GOAL_SHOT_TOPSPIN_CALIBRATION);
    public static final ShooterCalibrationPair TARMAC_SHOT_CALIBRATION_PAIR = new ShooterCalibrationPair("Tarmac Shot", TARMAC_SHOT_BACKSPIN_CALIBRATION, TARMAC_SHOT_TOPSPIN_CALIBRATION);
    public static final ShooterCalibrationPair AUTO_RADIUS_SHOT_CALIBRATION_PAIR = new ShooterCalibrationPair("Auto Radius Shot", AUTO_RADIUS_SHOT_BACKSPIN_CALIBRATION, AUTO_RADIUS_SHOT_TOPSPIN_CALIBRATION);
    public static final ShooterCalibrationPair LAUNCHPAD_SHOT_CALIBRATION_PAIR = new ShooterCalibrationPair("Launchpad Shot", LAUNCHPAD_SHOT_BACKSPIN_CALIBRATION, LAUNCHPAD_SHOT_TOPSPIN_CALIBRATION);

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
    public static final double CONVEYANCE_TWO_FULL_SPEED = .25;



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
