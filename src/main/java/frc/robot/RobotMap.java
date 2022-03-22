// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class RobotMap {

    //FEEDER
    public static final int FEEDER_MOTOR = 43;
    public static final int FEEDER_CONVEYANCE_MOTOR = 42;

    //CONVEYANCE
    public static final int CONVEYANCE_MOTOR = 41;

    //CLIMBER
    public static final int LEFT_CLIMBER_MOTOR = 51;
    public static final int RIGHT_CLIMBER_MOTOR = 52;
    public static final int CLIMBER_EXTENSION_SOLENOID = 4;
    public static final int CLIMBER_RETRACTION_SOLENOID = 5;
    
    // public static final int LEFT_CLIMBER_SOLENOID = 2;
    // public static final int RIGHT_CLIMBER_SOLENOID = 3;

    //SHOOTER
    public static final int SHOOTER_BACKSPIN_MOTOR = 61;
    public static final int SHOOTER_TOPSPIN_MOTOR = 62;

    //TURRET
    public static final int TURRET_MOTOR = 11;
    //public static final int TURRET_RESET_SENSOR = ;
    //public static final int TURRET_CLOCKWISE_SENSOR = ;
    //public static final int TURRET_COUNTERCLOCKWISE_SENSOR = ;

    //INTAKE 
    public static final int INTAKE_EXTEND_LEFT_SOLENOID = 0;
    public static final int INTAKE_RETRACT_LEFT_SOLENOID = 1;
    public static final int INTAKE_EXTEND_RIGHT_SOLENOID = 2;
    public static final int INTAKE_RETRACT_RIGHT_SOLENOID = 3; 
    
    // DRIVETRAIN
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5715; // The left-to-right distance between the drivetrain wheels
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5715; // The front-to-back distance between the drivetrain wheels.

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 8;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 24;
    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(73.301); // practice
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = Math.toRadians(127); // competition

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22;
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(320.625); // practice
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(79); // competition

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 2;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 21;
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(191.338); // practice
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = Math.toRadians(57); // competition

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 23;
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(201.885); // practice
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(274); // competition

    //SENSORS
    public static final int SENSOR_A_CHANNEL = 0; 
    public static final int SENSOR_B_CHANNEL = 1; 
    public static final int FIRST_SENSOR_CHANNEL = 2; // currently does not have correct channel number

    
    //LIMELIGHT
    public static final String CAMERA_NAME = "cam0";
}
