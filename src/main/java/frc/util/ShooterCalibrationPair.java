// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util;

/** Add your docs here. */
public class ShooterCalibrationPair {
    public String _name;
    public ShooterCalibration _backspinMotorCalibration;
    public ShooterCalibration _topspinMotorCalibration;
    
    public ShooterCalibrationPair(String name, ShooterCalibration backspinMotorCalibration, ShooterCalibration topspinMotorCalibration) {
        this._name = name;
        this._backspinMotorCalibration = backspinMotorCalibration;
        this._topspinMotorCalibration = topspinMotorCalibration;
    }
}
