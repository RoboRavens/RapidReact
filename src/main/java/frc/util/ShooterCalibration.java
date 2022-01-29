/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

public class ShooterCalibration {

    public String name;
    public int targetRPM;
    public double kF;
    public double kP;
    public double kI;
    public double kD;
    public int upperBoundBuffer;
    public int lowerBoundBuffer;

    /**
     * A "shot" object, storing all values needed for revving motors to the shot rpm with PID
     * @param name Name of the shot
     * @param RPM Target RPM to rev to
     * @param kF F value for PID control
     * @param kP P value for PID control
     * @param kI I value for PID control
     * @param kD D value for PID control
     */
    public ShooterCalibration(String name, int RPM, double kF, double kP, double kI, double kD) {
        this.name = name;
        this.targetRPM = RPM;
        this.kF = kF;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}