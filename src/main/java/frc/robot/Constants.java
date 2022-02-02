// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    // DRIVETRAIN
        /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5715; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5715; // FIXME Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID

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
    public static final double CONVEYANCE_FULL_SPEED = 1;
    public static final double CONVEYANCE_FULL_SPEED_REVERSE = -1;
    public static final double CONVEYANCE_NORMAL_SPEED = 0.75;
    public static final double CONVEYANCE_NORMAL_REVERSE_SPEED = -0.75;
    public static final double CONVEYANCE_STOP = 0;
    public static final double CONVEYANCE_FEEDER_SPEED = 0.75;
	public static final double CONVEYANCE_FEEDER_STOP = 0;
	public static final double CONVEYANCE_REVERSE_FEEDER = -.75;
}
