// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

import java.sql.Driver;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.controls.AxisCode;
import frc.controls.ButtonCode;
import frc.controls.Gamepad;
import frc.ravenhardware.RavenBlinkin;
import frc.robot.commands.*;
import frc.robot.commands.Auto.TwoBallHangarAutoCommand;
import frc.robot.commands.shooter.*;
import frc.robot.commands.turret.*;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  
  public static final Joystick JOYSTICK = new Joystick(0);
  public static final Gamepad GAMEPAD = new Gamepad(JOYSTICK);
  private Gamepad OP_PAD = new Gamepad(1);
  private Gamepad OP_PAD2 = new Gamepad(2);

  
  
  public static final DriveTrainSubsystemBase DRIVE_TRAIN_SUBSYSTEM = new DriveTrainSubsystem();
  public static final ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem();
  public static final IntakeExtenderSubsystem INTAKE_SUBSYSTEM = new IntakeExtenderSubsystem();
  public static final ConveyanceSubsystem CONVEYANCE_SUBSYSTEM = new ConveyanceSubsystem();
  public static final IntakeExtendCommand IntakeExtend = new IntakeExtendCommand();
  public static final ShooterStartCommand SHOOTER_START_COMMAND = new ShooterStartCommand();
  public static final ShooterStopCommand SHOOTER_STOP_COMMAND = new ShooterStopCommand();
  public static final IntakeRetractCommand IntakeRetract = new IntakeRetractCommand();
  public static final ClimberSubsystem CLIMBER_SUBSYSTEM = new ClimberSubsystem();
  public static final FeederSubsystem FEEDER_SUBSYSTEM = new FeederSubsystem();
  public static final TurretSwivelSubsystem TURRET_SWIVEL_SUBSYSTEM = new TurretSwivelSubsystem();
  public static final ConveyanceCollectCommand CONVEYANCE_COLLECT_COMMAND = new ConveyanceCollectCommand();
  public static final ConveyanceEjectCommand CONVEYANCE_EJECT_COMMAND = new ConveyanceEjectCommand();
  public static final FeederEjectCommand FeederEject = new FeederEjectCommand();
  public static final FeederSafetyReverseCommand FeederSafetyReverse = new FeederSafetyReverseCommand(Constants.FEEDER_SAFETY_REVERSE_DURATION);
  public static final ConveyanceIndexCommand CONVEYANCE_INDEX_COMMAND = new ConveyanceIndexCommand();
  public static final FeederShootCommand FeederShoot = new FeederShootCommand();
  public static final FeederIndexCommand FeederIndex = new FeederIndexCommand();
  public static final ShooterLowGoalCommand SHOOTER_LOW_GOAL_PID_COMMAND = new ShooterLowGoalCommand();
  public static final ShooterTarmacCommand SHOOTER_TARMAC_PID_COMMAND = new ShooterTarmacCommand();
  public static final ShooterLaunchpadCommand SHOOTER_LAUNCH_PAD_PID_COMMAND = new ShooterLaunchpadCommand();
  public static final FeederCollectCommand FeederCollect = new FeederCollectCommand();
  public static final ClimberDefaultBrakeCommand climberDefaultBrake = new ClimberDefaultBrakeCommand();
  public static final CompressorSubsystem COMPRESSOR_SUBSYSTEM = new CompressorSubsystem();
  public static final LimelightSubsystem LIMELIGHT_SUBSYSTEM = new LimelightSubsystem();
  public static final TurretAimAtTargetCommand TURRET_AIM_AT_TARGET = new TurretAimAtTargetCommand();
  public static final TurretFlipCommand TURRET_FLIP = new TurretFlipCommand();
  public static final TurretSeekCommand TURRET_SEEK = new TurretSeekCommand();
  public static final RavenBlinkin RAVEN_BLINKIN_3 = new RavenBlinkin(3);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    var driveTrainDefaultCommand = new DrivetrainDefaultCommand();
    DRIVE_TRAIN_SUBSYSTEM.setDefaultCommand(driveTrainDefaultCommand);
    //SHOOTER_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> SHOOTER_SUBSYSTEM.defaultCommand(), SHOOTER_SUBSYSTEM));

    SHOOTER_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> SHOOTER_SUBSYSTEM.defaultCommand(), SHOOTER_SUBSYSTEM));
    TURRET_SWIVEL_SUBSYSTEM.setDefaultCommand(TURRET_AIM_AT_TARGET);
    FEEDER_SUBSYSTEM.setDefaultCommand(FeederIndex);
    CLIMBER_SUBSYSTEM.setDefaultCommand(climberDefaultBrake);
    CONVEYANCE_SUBSYSTEM.setDefaultCommand(CONVEYANCE_INDEX_COMMAND);
    configureButtonBindings();
     LIMELIGHT_SUBSYSTEM.turnLEDOff();
    CameraServer.startAutomaticCapture();
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

    if (GAMEPAD.getAxisIsPressed(AxisCode.LEFTTRIGGER)) {
      Robot.LIMELIGHT_SUBSYSTEM.turnLEDOn();
    } else {
      Robot.LIMELIGHT_SUBSYSTEM.turnLEDOff();
    }
    
    if(Robot.CONVEYANCE_SUBSYSTEM.getConveyanceHasBall() == false && Robot.FEEDER_SUBSYSTEM.getFeederHasBall() == false) {
      RavenBlinkin.solidRed();
    } else if(Robot.CONVEYANCE_SUBSYSTEM.getConveyanceHasBall() == true | Robot.FEEDER_SUBSYSTEM.getFeederHasBall() == true) {
      RavenBlinkin.solidYellow();
    }else if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceHasBall() == true && Robot.FEEDER_SUBSYSTEM.getFeederHasBall() == true ){
      RavenBlinkin.solidGreen();
    }
  
  
  }
   /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_autonomousCommand = TwoBallHangarAutoCommand.get();
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
    SHOOTER_SUBSYSTEM.resetShotCount();

    // Stop any autonomous command that might still be running.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
     Timer.getFPGATimestamp();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (Timer.getFPGATimestamp() <= 60) {
      RAVEN_BLINKIN_3.blinkGreen();
    } else if (Timer.getFPGATimestamp() <= 30) {
      RAVEN_BLINKIN_3.blinkYellow();
    } else if (Timer.getFPGATimestamp() <= 15) {
      RAVEN_BLINKIN_3.blinkRed();
    }
   }

   public void configureButtonBindings() {
    GAMEPAD.getButton(ButtonCode.RIGHTBUMPER).whileHeld(CONVEYANCE_COLLECT_COMMAND);
    //GAMEPAD.getButton(ButtonCode.B).whileHeld(new SequentialCommandGroup(new WaitCommand(.15), SHOOTER_START_COMMAND));
    GAMEPAD.getButton(ButtonCode.LEFTBUMPER).whileHeld(CONVEYANCE_EJECT_COMMAND);
    //GAMEPAD.getButton(ButtonCode.B).whenPressed(FeederSafetyReverse);
    OP_PAD2.getButton(ButtonCode.SHOOTER_OVERRIDE).whileHeld(FeederShoot);
    //GAMEPAD.getButton(ButtonCode.BACK).whenHeld(TURRET_FLIP);
    //GAMEPAD.getButton(ButtonCode.START).whenHeld(TURRET_SEEK);
    GAMEPAD.getButton(ButtonCode.A).whileHeld(FeederCollect);
    GAMEPAD.getButton(ButtonCode.B).whileHeld(FeederEject);
    //GAMEPAD.getButton(ButtonCode.Y).whenPressed(FeederSafetyReverse);
    OP_PAD.getButton(ButtonCode.SHOOTER_LAUNCH_PAD_SHOT).whenPressed(SHOOTER_LAUNCH_PAD_PID_COMMAND);
    OP_PAD.getButton(ButtonCode.SHOOTER_TARMAC_SHOT).whenPressed(SHOOTER_TARMAC_PID_COMMAND);
    OP_PAD.getButton(ButtonCode.SHOOTER_LOW_GOAL_SHOT).whenPressed(SHOOTER_LOW_GOAL_PID_COMMAND);
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
