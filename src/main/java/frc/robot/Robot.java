// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.controls.AxisCode;
import frc.controls.ButtonCode;
import frc.controls.Gamepad;
import frc.robot.commands.ConveyanceCollectCommand;
import frc.robot.commands.ConveyanceEjectCommand;
import frc.robot.commands.ConveyanceIndexCommmand;
import frc.robot.commands.DrivetrainDefaultCommand;
import frc.robot.commands.FeederEjectCommand;
import frc.robot.commands.FeederIndexCommand;
import frc.robot.commands.FeederSafetyReverseCommand;
import frc.robot.commands.FeederShootCommand;
import frc.robot.commands.IntakeExtendCommand;
import frc.robot.commands.IntakeRetractCommand;
import frc.robot.commands.shooter.ShooterStartCommand;
import frc.robot.commands.turret.TurretAimAtTargetCommand;
import frc.robot.commands.turret.TurretFlipCommand;
import frc.robot.commands.turret.TurretResetEncoderCommand;
import frc.robot.commands.turret.TurretSeekCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyanceSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeExtenderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSwivelSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  public static final Gamepad GAMEPAD = new Gamepad(0);

  

  

  
  //public static final DriveTrainSubsystem DRIVE_TRAIN_SUBSYSTEM = new DriveTrainSubsystem();
  public static final ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem();
  public static final IntakeExtenderSubsystem INTAKE_SUBSYSTEM = new IntakeExtenderSubsystem();
  public static final ConveyanceSubsystem CONVEYANCE_SUBSYSTEM = new ConveyanceSubsystem();
  public static final TurretSwivelSubsystem TURRET_SWIVEL_SUBSYSTEM = new TurretSwivelSubsystem();
  public static final IntakeExtendCommand IntakeExtend = new IntakeExtendCommand();
  public static final ShooterStartCommand ShooterStart = new ShooterStartCommand();
  public static final IntakeRetractCommand IntakeRetract = new IntakeRetractCommand();
  public static final ClimberSubsystem CLIMBER_SUBSYSTEM = new ClimberSubsystem();
  public static final FeederSubsystem FEEDER_SUBSYSTEM = new FeederSubsystem();
  public static final ConveyanceCollectCommand CONVEYANCE_COLLECT_COMMAND = new ConveyanceCollectCommand();
  public static final ConveyanceEjectCommand CONVEYANCE_EJECT_COMMAND = new ConveyanceEjectCommand();
  //public static final FeederEjectCommand FeederEject = new FeederEjectCommand();
  public static final FeederSafetyReverseCommand FeederSafetyReverse = new FeederSafetyReverseCommand(Constants.FEEDER_SAFETY_REVERSE_DURATION);
  public static final ConveyanceIndexCommmand CONVEYANCE_INDEX_COMMAND = new ConveyanceIndexCommmand();
  public static final FeederShootCommand FeederShoot = new FeederShootCommand();
  public static final FeederIndexCommand FeederIndex = new FeederIndexCommand();

  public static final TurretResetEncoderCommand TURRET_RESET = new TurretResetEncoderCommand();
  public static final TurretFlipCommand TURRET_FLIP = new TurretFlipCommand();
  public static final TurretSeekCommand TURRET_SEEK = new TurretSeekCommand();
  public static final TurretAimAtTargetCommand TURRET_AIM_AT_TARGET = new TurretAimAtTargetCommand();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    //DRIVE_TRAIN_SUBSYSTEM.setDefaultCommand(new DrivetrainDefaultCommand(DRIVE_TRAIN_SUBSYSTEM, GAMEPAD));
    SHOOTER_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> SHOOTER_SUBSYSTEM.defaultCommand(), SHOOTER_SUBSYSTEM));
    TURRET_SWIVEL_SUBSYSTEM.setDefaultCommand(TURRET_AIM_AT_TARGET);
    TURRET_SWIVEL_SUBSYSTEM.setEncoder(0);

    FEEDER_SUBSYSTEM.setDefaultCommand(FeederIndex);
    configureButtonBindings();
  }
  
  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  
  }




  public void configureButtonBindings() {
    GAMEPAD.getButton(ButtonCode.RIGHTBUMPER).whileHeld(CONVEYANCE_COLLECT_COMMAND);
    GAMEPAD.getButton(ButtonCode.Y).whileHeld(ShooterStart);
    GAMEPAD.getButton(ButtonCode.LEFTBUMPER).whileHeld(CONVEYANCE_EJECT_COMMAND);
    //GAMEPAD.getButton(ButtonCode.A).whileHeld(FeederEject);
    GAMEPAD.getButton(ButtonCode.B).whenPressed(FeederSafetyReverse);
    GAMEPAD.getButton(ButtonCode.A).whileHeld(FeederShoot);

    //TURRET SWIVEL
    GAMEPAD.getButton(ButtonCode.BACK).whenPressed(TURRET_FLIP);
    GAMEPAD.getButton(ButtonCode.START).whenPressed(TURRET_SEEK);
    GAMEPAD.getButton(ButtonCode.RIGHTSTICK).whenPressed(TURRET_RESET);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}


}
