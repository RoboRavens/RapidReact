// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class RobotMap {

    //MOTORS

    public static final int CONVEYANCE_MOTOR_TWO = 25;
    public static final int CONVEYANCE_MOTOR_ONE = 26;
    public static final int CONVEYANCE_WHEEL = 27;
    
    //Set to -1 just so we don't conflict, but add the right values asap!
    public static final int SHOOTER_MOTOR_1 = 41;
    public static final int SHOOTER_MOTOR_2 = 42;

    

    //INTAKE
     
     public static final int INTAKE_EXTEND_SOLENOID = 0;
     public static final int INTAKE_RETRACT_SOLENOID = 1;
    
   
     //SENSORS
    public static final int SENSOR_A_CHANNEL = 1;
    public static final int SENSOR_B_CHANNEL = 1;
    
    // DRIVETRAIN
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5715; // The left-to-right distance between the drivetrain wheels
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5715; // The front-to-back distance between the drivetrain wheels.

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 8;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 24;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(253.301);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(140.625);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 2;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 21;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(11.338);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 23;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(21.885);
    
   

}
