
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;


import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {

  private TalonFX _intakeMotor;
  private Solenoid _intakeExtend;
  private Solenoid _intakeRetract;

  public IntakeSubsystem() {
    this.initialize();
    _intakeMotor = new TalonFX(RobotMap.INTAKE_MOTOR);
    _intakeExtend = new Solenoid(RobotMap.INTAKE_EXTEND_SOLENOID, 0);
    _intakeRetract = new Solenoid(RobotMap.INTAKE_RETRACT_SOLENOID, 0);
  }

  public void initialize() {
  }

  public void periodic() {

  }

  public void collect() {
  
  }

  public void spit() {
    
  }

  public void runAtPower() {
    
  }

  public void stop() {
    
  }

  public void extend() {
    
  }

  public void retract() {
    
  }

  public void stopAndRetract() {
   
  }

  public void defaultCommand() {
   
  }
}