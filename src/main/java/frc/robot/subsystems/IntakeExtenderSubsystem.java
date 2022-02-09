/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;



import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeExtenderSubsystem extends SubsystemBase {

 
  private Solenoid _intakeExtend;
  private Solenoid _intakeRetract;
  private DigitalInput _conveyanceSensorA;

  public IntakeExtenderSubsystem() {
    this.initialize();
    //Intake Solenoid stuff
    //_intakeExtend = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_EXTEND_SOLENOID);
    //_intakeRetract = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_RETRACT_SOLENOID);
    //_conveyanceSensorA = new DigitalInput(RobotMap.SENSOR_A_CHANNEL);
  }

  public void initialize() {
  }

  public void periodic() {

  }

  
  

  public void stop() {
    
  }

  public void extend() {
    //_intakeRetract.set(false);
    //_intakeExtend.set(true);
  }

  public void retract() {
    //_intakeRetract.set(true);
    //_intakeExtend.set(false);
  }

  public void stopAndRetract() {
    
  }

  public void defaultCommand() {
    
  }

  public boolean getConveyanceSensorAReading() {
    return false; //_conveyanceSensorA.get();
  }

  public boolean getConveyanceSensorBReading() {
    return false; //this.getConveyanceSensorBReading();
  }
}