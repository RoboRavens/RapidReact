// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util;

/** Binds a backspin and topspin calibration together, with the shot name. */
public class ShooterCalibrationPair {
    public final String _name;
    public final ShooterCalibration _backspinMotorCalibration;
    public final ShooterCalibration _topspinMotorCalibration;
    
    public ShooterCalibrationPair(String name, ShooterCalibration backspinMotorCalibration, ShooterCalibration topspinMotorCalibration) {
        this._name = name;
        this._backspinMotorCalibration = backspinMotorCalibration;
        this._topspinMotorCalibration = topspinMotorCalibration;
    }
}
