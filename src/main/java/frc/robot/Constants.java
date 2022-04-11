// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

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
    // LIVONIA AUTO CONSTANTS
    public static final double TWO_BALL_SHOOTER_DURATION = 1.5;
    public static final double THIRD_BALL_SHOOTER_DURATION = 1;


    // TALONFX
    public static final double TALONFX_TICKS_PER_REVOLUTION = 2048;
    public static final double TALON_TPS_TO_RPM = 600;
    public static final double TALON_VELOCITY_TO_RPM = TALON_TPS_TO_RPM / TALONFX_TICKS_PER_REVOLUTION;
    public static final double TALON_RPM_TO_VELOCITY = 1 / TALON_VELOCITY_TO_RPM;

    // CONTROLS
    public static final double JOYSTICK_DEADBAND = .05;
    public static final double DRIVE_MAX_TURN_RADIANS_PER_SECOND = 2;

    // COLOR SENSE
    public static final double BALL_COLOR_THRESHOLD_ENTRY = 400;
    public static final double BALL_COLOR_THRESHOLD_EXIT = 400;
  
    // SHOOTER RPM ALLOWANCE TESTED AT 50 for TELEOP BUT SHOULD NOT MATTER
    //SHOOTER
    public static final int SHOOTER_IDX = 0;
    public static final int SHOOTER_TIMEOUT_MS = 100;
    public static final int SHOOTER_TARGET_ALLOWANCE = 50; //Amount of RPM offset to consider still "on target"

    public static final double BACKSPIN_GEAR_RATIO = 16.0 / 36.0;
    public static final double TOPSPIN_GEAR_RATIO = 16.0 / 24.0;


    public static final double SHOOTER_CONVEYANCE_EJECTING_ARBITRARY_FEED_FORWARD = 0.01;
    public static final double SHOOTER_CONVEYANCE_INDEXING_ARBITRARY_FEED_FORWARD = 0.01;
    public static final double SHOOTER_CONVEYANCE_INTAKING_ARBITRARY_FEED_FORWARD = 0.01;

    //public static final int SHOOTER_BACKSPIN_VEL_TO_RPM = 8192 / 600;
    //public static final int SHOOTER_TOPSPIN_VEL_TO_RPM = SHOOTER_BACKSPIN_VEL_TO_RPM * 2/1; // Big Wheel:Small Wheel

    /* SHOOTER SETPOINTS AS TUNED MARCH 16th WITH ONLY KF:
        Tarmac shot:
            Setpoint 2460
            Actual 2404
            With shot sequence: 3050 up to 2200


        Launch pad shot:
            Setpoint 3k
            Actual: Topspin 2935, backspin 2825
            With shot sequence: top dip to 2500, normalize at 2700. Back dip to 24-2500, normalize at 2590
    */


// NEW CONSTANTS WITH kF based on feeder running and AFF added
/*
    // All shots need to be tuned.
    public static final int LOW_GOAL_BACKSPIN_RPM = 1000;
    public static final double LOW_GOAL_BACKSPIN_KF = 0.0452 * 1.1;  
 //   public static final double LOW_GOAL_BACKSPIN_KF = 0.0;
    public static final double LOW_GOAL_BACKSPIN_KP = 0.0;//0.17;
    public static final double LOW_GOAL_BACKSPIN_KI = 0;
    public static final double LOW_GOAL_BACKSPIN_KD = 0.0;
    
    public static final double LOW_GOAL_BACKSPIN_VOLTAGE_CONTROL_SETPOINT = LOW_GOAL_BACKSPIN_KF * 12; 

    public static final int LOW_GOAL_TOPSPIN_RPM = 1000;
    public static final double LOW_GOAL_TOPSPIN_KF = 0.047 * 1.1;
//    public static final double LOW_GOAL_TOPSPIN_KF = 0.0;
    public static final double LOW_GOAL_TOPSPIN_KP = 0.0;//0.17;
    public static final double LOW_GOAL_TOPSPIN_KI = 0;
    public static final double LOW_GOAL_TOPSPIN_KD = 0.0;

    public static final double LOW_GOAL_TOPSPIN_VOLTAGE_CONTROL_SETPOINT = LOW_GOAL_TOPSPIN_KF * 12;
// was 2460
    public static final int TARMAC_BACKSPIN_RPM = 2225;
    public static final double TARMAC_BACKSPIN_KF = 0.0449 * 1.1;
//    public static final double TARMAC_BACKSPIN_KF = 0.0;
    public static final double TARMAC_BACKSPIN_KP = .0;//0.21;
    public static final double TARMAC_BACKSPIN_KI = 0.0;
    public static final double TARMAC_BACKSPIN_KD = 0.0;

    public static final double TARMAC_BACKSPIN_VOLTAGE_CONTROL_SETPOINT = TARMAC_BACKSPIN_KF * 12;
// was 2460
    public static final int TARMAC_TOPSPIN_RPM = 2225;
    public static final double TARMAC_TOPSPIN_KF = 0.046 * 1.1;
//    public static final double TARMAC_TOPSPIN_KF = 0.0;
    public static final double TARMAC_TOPSPIN_KP = 0.0;//0.21;
    public static final double TARMAC_TOPSPIN_KI = 0.0;
    public static final double TARMAC_TOPSPIN_KD = 0.0;

    public static final double TARMAC_TOPSPIN_VOLTAGE_CONTROL_SETPOINT = TARMAC_TOPSPIN_KF * 12;

    public static final int AUTO_RADIUS_BACKSPIN_RPM = 2425;
//    public static final double AUTO_RADIUS_BACKSPIN_KF = 0.0;
    public static final double AUTO_RADIUS_BACKSPIN_KF = 0.0449 * 1.1;
    public static final double AUTO_RADIUS_BACKSPIN_KP = 0.0;//0.21;
    public static final double AUTO_RADIUS_BACKSPIN_KI = 0.0;
    public static final double AUTO_RADIUS_BACKSPIN_KD = 0.0;

    
    public static final double AUTO_RADIUS_BACKSPIN_VOLTAGE_CONTROL_SETPOINT = AUTO_RADIUS_BACKSPIN_KF * 12;

    public static final int AUTO_RADIUS_TOPSPIN_RPM = 2425;
    public static final double AUTO_RADIUS_TOPSPIN_KF = 0.046 * 1.1;
    //public static final double AUTO_RADIUS_TOPSPIN_KF = 0.0;
    public static final double AUTO_RADIUS_TOPSPIN_KP = 0.0;//0.21;
    public static final double AUTO_RADIUS_TOPSPIN_KI = 0.0;
    public static final double AUTO_RADIUS_TOPSPIN_KD = 0.0;

    public static final double AUTO_RADIUS_TOPSPIN_VOLTAGE_CONTROL_SETPOINT = AUTO_RADIUS_TOPSPIN_KF * 12;

    // LAUNCHPAD WAS 3000

    public static final int LAUNCHPAD_BACKSPIN_RPM = 2675;
   // public static final double LAUNCHPAD_BACKSPIN_KF = 0.0;
    public static final double LAUNCHPAD_BACKSPIN_KF = 0.0449 * 1.1;
    public static final double LAUNCHPAD_BACKSPIN_KP = 0.0;//0.22;
    public static final double LAUNCHPAD_BACKSPIN_KI = 0.0;
    public static final double LAUNCHPAD_BACKSPIN_KD = 0.0;

    public static final double LAUNCHPAD_BACKSPIN_VOLTAGE_CONTROL_SETPOINT = LAUNCHPAD_BACKSPIN_KF * 12;
*/

public double wednesdayNightCoefficient = 1.08;



    // All shots need to be tuned.
    public static final int LOW_GOAL_BACKSPIN_RPM = 850;
    public static final double LOW_GOAL_BACKSPIN_KF = 0.0452;  
 //   public static final double LOW_GOAL_BACKSPIN_KF = 0.0;
    public static final double LOW_GOAL_BACKSPIN_KP = 0.15;//0.17;
    public static final double LOW_GOAL_BACKSPIN_KI = 0;
    public static final double LOW_GOAL_BACKSPIN_KD = 0.2;
    
    public static final double LOW_GOAL_BACKSPIN_VOLTAGE_CONTROL_SETPOINT = LOW_GOAL_BACKSPIN_KF * 12; 

    public static final int LOW_GOAL_TOPSPIN_RPM = 1650;
    public static final double LOW_GOAL_TOPSPIN_KF = 0.047;
//    public static final double LOW_GOAL_TOPSPIN_KF = 0.0;
    public static final double LOW_GOAL_TOPSPIN_KP = 0.15;//0.17;
    public static final double LOW_GOAL_TOPSPIN_KI = 0;
    public static final double LOW_GOAL_TOPSPIN_KD = 0.2;

    public static final double LOW_GOAL_TOPSPIN_VOLTAGE_CONTROL_SETPOINT = LOW_GOAL_TOPSPIN_KF * 12;
// was 2460
    public static final double TARMAC_BACKSPIN_RPM = 2225;
    public static final double TARMAC_BACKSPIN_KF = 0.0449 * 1.08;
//    public static final double TARMAC_BACKSPIN_KF = 0.0;
    public static final double TARMAC_BACKSPIN_KP = .350;//0.21;
    public static final double TARMAC_BACKSPIN_KI = 0;
    public static final double TARMAC_BACKSPIN_KD = 4.0;

    public static final double TARMAC_BACKSPIN_VOLTAGE_CONTROL_SETPOINT = TARMAC_BACKSPIN_KF * 12;
// was 2460
    public static final double TARMAC_TOPSPIN_RPM = 2225;
    public static final double TARMAC_TOPSPIN_KF = 0.046 * 1.08;
//    public static final double TARMAC_TOPSPIN_KF = 0.0;
    public static final double TARMAC_TOPSPIN_KP = .35;//0.21;
    public static final double TARMAC_TOPSPIN_KI = 0;
    public static final double TARMAC_TOPSPIN_KD = 4.0;

    public static final double TARMAC_TOPSPIN_VOLTAGE_CONTROL_SETPOINT = TARMAC_TOPSPIN_KF * 12;

    public static final double AUTO_RADIUS_BACKSPIN_RPM = 2425;
//    public static final double AUTO_RADIUS_BACKSPIN_KF = 0.0;
    public static final double AUTO_RADIUS_BACKSPIN_KF = 0.0449 * 1.08;
    public static final double AUTO_RADIUS_BACKSPIN_KP = 0.35;//0.21;
    public static final double AUTO_RADIUS_BACKSPIN_KI = 0;
    public static final double AUTO_RADIUS_BACKSPIN_KD = 4.0;

    
    public static final double AUTO_RADIUS_BACKSPIN_VOLTAGE_CONTROL_SETPOINT = AUTO_RADIUS_BACKSPIN_KF * 12;

    public static final double AUTO_RADIUS_TOPSPIN_RPM = 2425;
    public static final double AUTO_RADIUS_TOPSPIN_KF = 0.046 * 1.08;
    //public static final double AUTO_RADIUS_TOPSPIN_KF = 0.0;
    public static final double AUTO_RADIUS_TOPSPIN_KP = .35;//0.21;
    public static final double AUTO_RADIUS_TOPSPIN_KI = 0;
    public static final double AUTO_RADIUS_TOPSPIN_KD = 4.0;

    public static final double AUTO_RADIUS_TOPSPIN_VOLTAGE_CONTROL_SETPOINT = AUTO_RADIUS_TOPSPIN_KF * 12;

    // LAUNCHPAD WAS 3000

    public static final double LAUNCHPAD_BACKSPIN_RPM = 2950;
   // public static final double LAUNCHPAD_BACKSPIN_KF = 0.0;
    public static final double LAUNCHPAD_BACKSPIN_KF = 0.0449 * 1.08;
    public static final double LAUNCHPAD_BACKSPIN_KP = 0.35;//0.22;
    public static final double LAUNCHPAD_BACKSPIN_KI = 0;
    public static final double LAUNCHPAD_BACKSPIN_KD = 4.0;

    public static final double LAUNCHPAD_BACKSPIN_VOLTAGE_CONTROL_SETPOINT = LAUNCHPAD_BACKSPIN_KF * 12;

    public static final double LAUNCHPAD_TOPSPIN_RPM = 2950;
    public static final double LAUNCHPAD_TOPSPIN_KF = 0.046 * 1.08;
    //public static final double LAUNCHPAD_TOPSPIN_KF = 0.0;
    public static final double LAUNCHPAD_TOPSPIN_KP = .35;//0.22;
    public static final double LAUNCHPAD_TOPSPIN_KI = 0;
    public static final double LAUNCHPAD_TOPSPIN_KD = 4.0;

    public static final double LAUNCHPAD_TOPSPIN_VOLTAGE_CONTROL_SETPOINT = LAUNCHPAD_TOPSPIN_KF * 12;





// OLD CONSTANTS - AS OF Tuesday, March 29, 4 PM
/*
    // All shots need to be tuned.
    public static final int LOW_GOAL_BACKSPIN_RPM = 1000;
    public static final double LOW_GOAL_BACKSPIN_KF = 0.0452;  
 //   public static final double LOW_GOAL_BACKSPIN_KF = 0.0;
    public static final double LOW_GOAL_BACKSPIN_KP = 0.15;//0.17;
    public static final double LOW_GOAL_BACKSPIN_KI = 0;
    public static final double LOW_GOAL_BACKSPIN_KD = 0.2;
    
    public static final double LOW_GOAL_BACKSPIN_VOLTAGE_CONTROL_SETPOINT = LOW_GOAL_BACKSPIN_KF * 12; 

    public static final int LOW_GOAL_TOPSPIN_RPM = 1000;
    public static final double LOW_GOAL_TOPSPIN_KF = 0.047;
//    public static final double LOW_GOAL_TOPSPIN_KF = 0.0;
    public static final double LOW_GOAL_TOPSPIN_KP = 0.15;//0.17;
    public static final double LOW_GOAL_TOPSPIN_KI = 0;
    public static final double LOW_GOAL_TOPSPIN_KD = 0.2;

    public static final double LOW_GOAL_TOPSPIN_VOLTAGE_CONTROL_SETPOINT = LOW_GOAL_TOPSPIN_KF * 12;
// was 2460
    public static final int TARMAC_BACKSPIN_RPM = 2225;
    public static final double TARMAC_BACKSPIN_KF = 0.0449;
//    public static final double TARMAC_BACKSPIN_KF = 0.0;
    public static final double TARMAC_BACKSPIN_KP = .350;//0.21;
    public static final double TARMAC_BACKSPIN_KI = 0;
    public static final double TARMAC_BACKSPIN_KD = 4.0;

    public static final double TARMAC_BACKSPIN_VOLTAGE_CONTROL_SETPOINT = TARMAC_BACKSPIN_KF * 12;
// was 2460
    public static final int TARMAC_TOPSPIN_RPM = 2225;
    public static final double TARMAC_TOPSPIN_KF = 0.046;
//    public static final double TARMAC_TOPSPIN_KF = 0.0;
    public static final double TARMAC_TOPSPIN_KP = .35;//0.21;
    public static final double TARMAC_TOPSPIN_KI = 0;
    public static final double TARMAC_TOPSPIN_KD = 4.0;

    public static final double TARMAC_TOPSPIN_VOLTAGE_CONTROL_SETPOINT = TARMAC_TOPSPIN_KF * 12;

    public static final int AUTO_RADIUS_BACKSPIN_RPM = 2425;
//    public static final double AUTO_RADIUS_BACKSPIN_KF = 0.0;
    public static final double AUTO_RADIUS_BACKSPIN_KF = 0.0449;
    public static final double AUTO_RADIUS_BACKSPIN_KP = 0.35;//0.21;
    public static final double AUTO_RADIUS_BACKSPIN_KI = 0;
    public static final double AUTO_RADIUS_BACKSPIN_KD = 4.0;

    
    public static final double AUTO_RADIUS_BACKSPIN_VOLTAGE_CONTROL_SETPOINT = AUTO_RADIUS_BACKSPIN_KF * 12;

    public static final int AUTO_RADIUS_TOPSPIN_RPM = 2425;
    public static final double AUTO_RADIUS_TOPSPIN_KF = 0.046;
    //public static final double AUTO_RADIUS_TOPSPIN_KF = 0.0;
    public static final double AUTO_RADIUS_TOPSPIN_KP = .35;//0.21;
    public static final double AUTO_RADIUS_TOPSPIN_KI = 0;
    public static final double AUTO_RADIUS_TOPSPIN_KD = 4.0;

    public static final double AUTO_RADIUS_TOPSPIN_VOLTAGE_CONTROL_SETPOINT = AUTO_RADIUS_TOPSPIN_KF * 12;

    // LAUNCHPAD WAS 3000

    public static final int LAUNCHPAD_BACKSPIN_RPM = 2675;
   // public static final double LAUNCHPAD_BACKSPIN_KF = 0.0;
    public static final double LAUNCHPAD_BACKSPIN_KF = 0.0449;
    public static final double LAUNCHPAD_BACKSPIN_KP = 0.35;//0.22;
    public static final double LAUNCHPAD_BACKSPIN_KI = 0;
    public static final double LAUNCHPAD_BACKSPIN_KD = 4.0;

    public static final double LAUNCHPAD_BACKSPIN_VOLTAGE_CONTROL_SETPOINT = LAUNCHPAD_BACKSPIN_KF * 12;

*/



/*

Y-offset constants in order:
	Minimum range (close to hub): .29
	Tarmac shots
	Cutoff between tarmac and cargo shots: -6
	Cargo shots
	Cutoff between cargo and launchpad shots: -9.2
	Launchpad shots
	Maximum range launchpad shot: -11.6

*/
/*
    public static final int LAUNCHPAD_TOPSPIN_RPM = 2675;
    public static final double LAUNCHPAD_TOPSPIN_KF = 0.046;
    //public static final double LAUNCHPAD_TOPSPIN_KF = 0.0;
    public static final double LAUNCHPAD_TOPSPIN_KP = .35;//0.22;
    public static final double LAUNCHPAD_TOPSPIN_KI = 0;
    public static final double LAUNCHPAD_TOPSPIN_KD = 4.0;

    public static final double LAUNCHPAD_TOPSPIN_VOLTAGE_CONTROL_SETPOINT = LAUNCHPAD_TOPSPIN_KF * 12;
*/


    // 3k RPM was about a 17 foot shot (16.6666 to middle of hub, slightly less to outer ring.)
    // Shot was not super consistent as of March 16.


    public static final ShooterCalibration LOW_GOAL_SHOT_BACKSPIN_CALIBRATION = new ShooterCalibration("Low Goal Shot", LOW_GOAL_BACKSPIN_RPM, LOW_GOAL_BACKSPIN_KF, LOW_GOAL_BACKSPIN_KP, LOW_GOAL_BACKSPIN_KI, LOW_GOAL_BACKSPIN_KD, LOW_GOAL_BACKSPIN_VOLTAGE_CONTROL_SETPOINT);
    public static final ShooterCalibration LOW_GOAL_SHOT_TOPSPIN_CALIBRATION = new ShooterCalibration("Low Goal Alt Shot", LOW_GOAL_TOPSPIN_RPM, LOW_GOAL_TOPSPIN_KF, LOW_GOAL_TOPSPIN_KP, LOW_GOAL_TOPSPIN_KI, LOW_GOAL_TOPSPIN_KD, LOW_GOAL_TOPSPIN_VOLTAGE_CONTROL_SETPOINT);
    
    public static final ShooterCalibration TARMAC_SHOT_BACKSPIN_CALIBRATION = new ShooterCalibration("Tarmac Shot", TARMAC_BACKSPIN_RPM, TARMAC_BACKSPIN_KF, TARMAC_BACKSPIN_KP, TARMAC_BACKSPIN_KI, TARMAC_BACKSPIN_KD, TARMAC_BACKSPIN_VOLTAGE_CONTROL_SETPOINT);
    public static final ShooterCalibration TARMAC_SHOT_TOPSPIN_CALIBRATION = new ShooterCalibration("Tarmac Alt Shot", TARMAC_TOPSPIN_RPM, TARMAC_TOPSPIN_KF, TARMAC_TOPSPIN_KP, TARMAC_TOPSPIN_KI, TARMAC_TOPSPIN_KD, TARMAC_TOPSPIN_VOLTAGE_CONTROL_SETPOINT);

    public static final ShooterCalibration AUTO_RADIUS_SHOT_BACKSPIN_CALIBRATION = new ShooterCalibration("Auto Radius Shot", AUTO_RADIUS_BACKSPIN_RPM, AUTO_RADIUS_BACKSPIN_KF, AUTO_RADIUS_BACKSPIN_KP, AUTO_RADIUS_BACKSPIN_KI, AUTO_RADIUS_BACKSPIN_KD, AUTO_RADIUS_BACKSPIN_VOLTAGE_CONTROL_SETPOINT);
    public static final ShooterCalibration AUTO_RADIUS_SHOT_TOPSPIN_CALIBRATION = new ShooterCalibration("Auto Radius Alt Shot", AUTO_RADIUS_TOPSPIN_RPM, AUTO_RADIUS_TOPSPIN_KF, AUTO_RADIUS_TOPSPIN_KP, AUTO_RADIUS_TOPSPIN_KI, AUTO_RADIUS_TOPSPIN_KD, AUTO_RADIUS_TOPSPIN_VOLTAGE_CONTROL_SETPOINT);

    public static final ShooterCalibration LAUNCHPAD_SHOT_BACKSPIN_CALIBRATION = new ShooterCalibration("Launchpad Shot", LAUNCHPAD_BACKSPIN_RPM, LAUNCHPAD_BACKSPIN_KF, LAUNCHPAD_BACKSPIN_KP, LAUNCHPAD_BACKSPIN_KI, LAUNCHPAD_BACKSPIN_KD, LAUNCHPAD_BACKSPIN_VOLTAGE_CONTROL_SETPOINT);
    public static final ShooterCalibration LAUNCHPAD_SHOT_TOPSPIN_CALIBRATION = new ShooterCalibration("Launchpad Alt Shot", LAUNCHPAD_TOPSPIN_RPM, LAUNCHPAD_TOPSPIN_KF, LAUNCHPAD_TOPSPIN_KP, LAUNCHPAD_TOPSPIN_KI, LAUNCHPAD_TOPSPIN_KD, LAUNCHPAD_TOPSPIN_VOLTAGE_CONTROL_SETPOINT);

    public static final ShooterCalibration DISABLED_SHOT_BACKSPIN_CALIBRATION = new ShooterCalibration("Disabled Shot", 0, 0, 0, 0, 0, 0);
    public static final ShooterCalibration DISABLED_SHOT_TOPSPIN_CALIBRATION = new ShooterCalibration("Disabled Alt Shot", 0, 0, 0, 0, 0, 0);

    public static final ShooterCalibrationPair LOW_GOAL_SHOT_CALIBRATION_PAIR = new ShooterCalibrationPair("Low Goal Shot", LOW_GOAL_SHOT_BACKSPIN_CALIBRATION, LOW_GOAL_SHOT_TOPSPIN_CALIBRATION);
    public static final ShooterCalibrationPair TARMAC_SHOT_CALIBRATION_PAIR = new ShooterCalibrationPair("Tarmac Shot", TARMAC_SHOT_BACKSPIN_CALIBRATION, TARMAC_SHOT_TOPSPIN_CALIBRATION);
    public static final ShooterCalibrationPair AUTO_RADIUS_SHOT_CALIBRATION_PAIR = new ShooterCalibrationPair("Auto Radius Shot", AUTO_RADIUS_SHOT_BACKSPIN_CALIBRATION, AUTO_RADIUS_SHOT_TOPSPIN_CALIBRATION);
    public static final ShooterCalibrationPair LAUNCHPAD_SHOT_CALIBRATION_PAIR = new ShooterCalibrationPair("Launchpad Shot", LAUNCHPAD_SHOT_BACKSPIN_CALIBRATION, LAUNCHPAD_SHOT_TOPSPIN_CALIBRATION);
    public static final ShooterCalibrationPair DISABLED_SHOT_CALIBRATION_PAIR = new ShooterCalibrationPair("Disabled shot", DISABLED_SHOT_BACKSPIN_CALIBRATION, DISABLED_SHOT_TOPSPIN_CALIBRATION);

    //TURRET SWIVEL
    public static boolean TURRET_ENABLED = true; // when false the turret will not move and the drivetrain will align shooter with the goal
    public static final int TURRET_IDX = 0;
    public static final int TURRET_TIMEOUT_MS = 100;
    public static final int TURRET_RANGE = 140; //Degrees of motion in either way (180 means full movement both ways)
    public static final double TURRET_GEARBOX_RATIO = 1.0 / 10.0; // Motor is one-tenth speed due to gearbox
    public static final double TURRET_LARGE_GEAR_RATIO = 10.0 / 128.0;
    public static final double ENCODER_TO_TURRET_RATIO = (TALONFX_TICKS_PER_REVOLUTION / 360.0) / TURRET_GEARBOX_RATIO * 3 / (180.0/155.464); // Multiply encoder by this to find angle of turret
    public static final double TURRET_AIM_ALLOWANCE = 2; //Degrees of allowance to say that the turret has "reached" its target
    public static final double TURRET_MISS_OFFSET = 35;
    public static final double TURRET_GEAR_RATIO = 1.0 / 10.0; // Motor is one-tenth speed due to gearbox
    public static final double TURRET_CLOCKWISE_HARDWARE_LIMIT = -153.707963;
    public static final double TURRET_COUNTER_CLOCKWISE_HARDWARE_LIMIT = 156.414829;

    public static final double TURRET_DEFAULT_KF = 0;
    public static final double TURRET_DEFAULT_KP = 0.6;
    //public static final double TURRET_DEFAULT_KP = 0.120; //0.075;
    public static final double TURRET_DEFAULT_KI = 0;
    public static final double TURRET_DEFAULT_KD = 0;

    public static final double TURRET_FLIP_KF = 0;
    public static final double TURRET_FLIP_KP = 0.00015;
    //public static final double TURRET_FLIP_KP = 0.120; //0.1;
    public static final double TURRET_FLIP_KI = 0;
    public static final double TURRET_FLIP_KD = 0;

    public static final int DESIRED_TURRET_TARGET_BUFFER = 1;

    public static final TurretCalibration TURRET_DEFAULT_PID = new TurretCalibration("Default", TURRET_DEFAULT_KF, TURRET_DEFAULT_KP, TURRET_DEFAULT_KI, TURRET_DEFAULT_KD);
    public static final TurretCalibration TURRET_FLIP_PID = new TurretCalibration("Flipping", TURRET_FLIP_KF, TURRET_FLIP_KP, TURRET_FLIP_KI, TURRET_FLIP_KD);

    //CONTROLS
    public static final double AXIS_IS_PRESSED_VALUE = .25;
    public static final double INTAKE_COLLECT_POWER_MAGNITUDE = 0.1;
    public static final double INTAKE_SPIT_POWER_MAGNITUDE = 0.1;

    // CONVEYANCE TWO & FEEDER
    // Not sure if these are the correct speeds for the conveyance motor (needs to be calibrated)
    public static final double CONVEYANCE_TWO_FULL_SPEED_REVERSE = .25;
    public static final double CONVEYANCE_TWO_NORMAL_SPEED = 0.25;
    public static final double CONVEYANCE_TWO_NORMAL_REVERSE_SPEED = -0.25;
    public static final double CONVEYANCE_TWO_STOP = 0;
    public static final double CONVEYANCE_TWO_FEEDER_SPEED = 1.0;
	public static final double CONVEYANCE_TWO_FEEDER_STOP = 0;
	public static final double CONVEYANCE_TWO_REVERSE_FEEDER = -1.0;
    public static final double CONVEYANCE_TWO_FULL_SPEED = .25;
    public static final double CONVEYANCE_TWO_SPEED_WHILE_INDEXING = .15;
    public static final double FEEDER_SAFETY_REVERSE_DURATION = .15;

    //CONVEYANCE ONE
    public static final double CONVEYANCE_ONE_FULL_SPEED_REVERSE = 1.0;
    public static final double CONVEYANCE_ONE_FULL_SPEED = -1; // Was -75, meant to be .75, but not changing functionality without testing.
    public static final double CONVEYANCE_ONE_INDEX_SPEED = -.25;
    public static final double CONVEYANCE_ONE_NORMAL_REVERSE_SPEED = -.25;
    public static final double CONVEYANCE_ONE_STOP = 0;

    // DRIVETRAIN PATHFINDING
    public static final double TRAJECTORY_CONFIG_MAX_VELOCITY_METERS_PER_SECOND = 1.5;
    public static final double TRAJECTORY_CONFIG_MAX_ACCELERATION_METERS_PER_SECOND = .6;
    public static final double TRAJECTORY_CONFIG_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
    public static final double TRAJECTORY_CONFIG_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND = Math.PI * .75;

    public static final double SWERVE_CONTROLLER_X_KP = 2;
    public static final double SWERVE_CONTROLLER_Y_KP = 2;
    public static final double SWERVE_CONTROLLER_ANGLE_KP = 4;

    private static final double SWERVE_WHEEL_DIAMETER = SdsModuleConfigurations.MK4_L1.getWheelDiameter();
    private static final double SWERVE_WHEEL_DIAMETER_INCREMENT_5BALL = 0.0003186; // reducing the wheel diameter by this increment adds one inch to the 5 ball auto
    private static final double SWERVE_5BALL_INCHES_OFFSET = 0; // positive number gets robot closer to human player station
    public static final double SWERVE_ODOMETRY_MULTIPLIER =
      (SWERVE_WHEEL_DIAMETER - (SWERVE_WHEEL_DIAMETER_INCREMENT_5BALL * SWERVE_5BALL_INCHES_OFFSET)) / SWERVE_WHEEL_DIAMETER;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints SWERVE_CONTROLLER_ANGULAR_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            TRAJECTORY_CONFIG_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, TRAJECTORY_CONFIG_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND);
    
    //CLIMBER
    public static final double CLIMBER_HOLD_POSITION_POWER_MAGNITUDE = 0; // .13
	public static final double CLIMBER_EXTEND_SLOW_POWER_MAGNITUDE = .5;
    public static final double CLIMBER_EXTEND_FAST_POWER_MAGNITUDE = 1;
    public static final double CLIMBER_EXTEND_SOWLY_POWER_MAGNITUDE = .2;
	public static final double CLIMBER_RETRACT_POWER_MAGNITUDE = -.4;
    public static final double CLIMBER_RETRACT_SOWLY_POWER_MAGNITUDE = -.2;
	public static final double CLIMBER_RETRACT_TO_LATCH_POWER_MAGNITUDE = .2;
    // public static final double CLIMBER_EXTEND_ENCODER_TARGET = 241251.000000 - 45320.000000;
    public static final double CLIMBER_EXTEND_ENCODER_TARGET = 260000.0;
    public static final double CLIMBER_ENCODER_ACCURACY_RANGE = 4000;
    public static final double CLIMBER_IS_EXTENDED_ENCODER_THRESHOLD = CLIMBER_EXTEND_ENCODER_TARGET / 5;

    
    //LimeLight
    public static final int FLOOR_TO_LIMELIGHT_LENS_HEIGHT = 37;  //Inches
    public static final int FLOOR_TO_TARGET_CENTER_HEIGHT = 0;
    public static final double CAMERA_ANGLE_OFFSET_FROM_HORIZONTAL = 0;
    public static final double LIMELIGHT_LENS_TO_ROBOT_CENTER_OFFSET_INCHES = 0;
    public static final double MINIMUM_DISTANCE_FROM_LIMELIGHT = 46.0;
	public static final double MAXIMUM_DISTANCE_FROM_LIMELIGHT = 240.0;
    public static final double LIMELIGHT_CENTERED_OFFSET = 1.2;
    public static final double LIMELIGHT_IS_ALIGNED_DEGREES = 1.5;
   
    public static final double MAX_LOW_GOAL_SHOT = .29;
	public static final double MAX_TARMAC_SHOT = -6;
	public static final double MAX_AUTO_RADIUS_SHOT = -9.2;
	public static final double MAX_LAUNCHPAD_SHOT = -11.6;
}
