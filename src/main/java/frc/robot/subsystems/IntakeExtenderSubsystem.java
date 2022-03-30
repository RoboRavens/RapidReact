package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeExtenderSubsystem extends SubsystemBase {
  /*
  private Solenoid _intakeLeftSideExtend;
  private Solenoid _intakeLeftSideRetract;
  private Solenoid _intakeRightSideExtend;
  private Solenoid _intakeRightSideRetract;
*/
  private DoubleSolenoid _intakeSolenoid;


  public IntakeExtenderSubsystem() {
    _intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_EXTEND_SOLENOID, RobotMap.INTAKE_RETRACT_SOLENOID);

/*
    _intakeLeftSideExtend = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_EXTEND_SOLENOID);
    _intakeLeftSideRetract = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_RETRACT_SOLENOID);
  
    _intakeRightSideExtend = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_EXTEND_RIGHT_SOLENOID);
    _intakeRightSideRetract = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_RETRACT_RIGHT_SOLENOID);
    */
  }

  public void extend() {
    _intakeSolenoid.set(Value.kForward);
    /*
    _intakeLeftSideRetract.set(false);
    _intakeRightSideRetract.set(false);
    _intakeLeftSideExtend.set(true);
    _intakeRightSideExtend.set(true);
    */
  }

  public void retract() {
    _intakeSolenoid.set(Value.kReverse);
    /*
    _intakeLeftSideRetract.set(true);
    _intakeRightSideRetract.set(true);
    _intakeLeftSideExtend.set(false);
    _intakeRightSideExtend.set(false);
    */
  }
}
