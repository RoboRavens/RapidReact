// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class RobotMap {

    //FEEDER
    public static final int FEEDER_MOTOR = 32;
    public static final int FEEDER_CONVEYANCE_MOTOR = 31;

    //CONVEYANCE
     public static final int CONVEYANCE_MOTOR = 11;

    //CLIMBER
    public static final int LEFT_CLIMBER_MOTOR = 51;
    public static final int RIGHT_CLIMBER_MOTOR = 52;
    public static final int LEFT_CLIMBER_SOLENOID = 2;
    public static final int RIGHT_CLIMBER_SOLENOID = 3;

    //SHOOTER
    public static final int SHOOTER_MOTOR_1 = 41;
    public static final int SHOOTER_MOTOR_2 = 42;

    //TURRET
    public static final int TURRET_MOTOR = LEFT_CLIMBER_MOTOR; //Set this to real value ASAP
    //public static final int TURRET_RESET_SENSOR = ;
    //public static final int TURRET_CLOCKWISE_SENSOR = ;
    //public static final int TURRET_COUNTERCLOCKWISE_SENSOR = ;

    //INTAKE 
     public static final int INTAKE_EXTEND_SOLENOID = 0;
     public static final int INTAKE_RETRACT_SOLENOID = 1; 
    
    //SENSORS
    public static final int SENSOR_A_CHANNEL = 0; 
    public static final int SENSOR_B_CHANNEL = 1; 

    
    //LIMELIGHT
    public static final String CAMERA_NAME = "cam0";
}
