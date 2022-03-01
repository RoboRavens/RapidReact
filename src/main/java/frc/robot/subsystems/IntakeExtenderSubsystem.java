
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ravenhardware.BufferedDigitalInput;
import frc.robot.RobotMap;

public class IntakeExtenderSubsystem extends SubsystemBase {
  private Solenoid _intakeExtend;
  private Solenoid _intakeRetract;

  public IntakeExtenderSubsystem() {
    //Intake Solenoid stuff
    _intakeExtend = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_EXTEND_SOLENOID);
    _intakeRetract = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_RETRACT_SOLENOID);
  }

  public void extend() {
    _intakeRetract.set(false);
    _intakeExtend.set(true);
  }

  public void retract() {
    _intakeRetract.set(true);
    _intakeExtend.set(false);
  }
}
