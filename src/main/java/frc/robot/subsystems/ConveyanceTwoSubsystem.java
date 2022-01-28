// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ConveyanceTwoSubsystem extends SubsystemBase {
   private TalonSRX conveyanceMotor = new TalonSRX(RobotMap.CONVEYANCE_MOTOR);

  /** Creates a new ExampleSubsystem. */
  public ConveyanceTwoSubsystem() {
      conveyanceMotor = new TalonSRX(RobotMap.CONVEYANCE_MOTOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
