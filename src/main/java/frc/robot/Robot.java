// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.controls.AxisCode;
import frc.controls.ButtonCode;
import frc.controls.Gamepad;
import frc.ravenhardware.BlinkinCalibrations;
import frc.ravenhardware.RavenBlinkin;
import frc.ravenhardware.RavenBlinkinPatternCodes;
import frc.ravenhardware.RavenPiColor;
import frc.ravenhardware.RavenPiColorSensor;
import frc.ravenhardware.RavenPiPosition;
import frc.robot.commands.Auto.FiveBallHps;
import frc.robot.commands.Auto.ThreeBallTarmacAutoCommand;
import frc.robot.commands.Auto.TwoBallAutoCommand;
import frc.robot.commands.Climber.ClimberDefaultBrakeCommand;
import frc.robot.commands.CommandGroup.JunkShotCommandGroup;
import frc.robot.commands.Conveyance.ConveyanceCollectCommand;
import frc.robot.commands.Conveyance.ConveyanceEjectCommand;
import frc.robot.commands.Conveyance.ConveyanceIndexCommand;
import frc.robot.commands.Conveyance.IntakeRetractCommand;
import frc.robot.commands.DriveTrain.DrivetrainDefaultCommand;
import frc.robot.commands.Feeder.FeederCollectCommand;
import frc.robot.commands.Feeder.FeederEjectCommand;
import frc.robot.commands.Feeder.FeederIndexCommand;
import frc.robot.commands.Feeder.FeederSafetyReverseCommand;
import frc.robot.commands.Feeder.FeederShootCommand;
import frc.robot.commands.Feeder.FeederShootOneBallCommand;
import frc.robot.commands.Feeder.FeederWheelReverseCommand;
import frc.robot.commands.shooter.ShooterAutoRadiusCommand;
import frc.robot.commands.shooter.ShooterLaunchpadCommand;
import frc.robot.commands.shooter.ShooterLowGoalCommand;
import frc.robot.commands.shooter.ShooterStartCommand;
import frc.robot.commands.shooter.ShooterStopCommand;
import frc.robot.commands.shooter.ShooterTarmacCommand;
import frc.robot.commands.turret.TurretAimAtTargetCommand;
import frc.robot.commands.turret.TurretFlipCommand;
import frc.robot.commands.turret.TurretSeekCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.ConveyanceSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DriveTrainSubsystemBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeExtenderSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSwivelSubsystem;
import frc.util.AutoMode;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private SendableChooser<AutoMode> _autoChooser = new SendableChooser<>();
  
  public static final Joystick JOYSTICK = new Joystick(0);
  public static final Gamepad GAMEPAD = new Gamepad(JOYSTICK);
  private Gamepad OP_PAD = new Gamepad(1);
  private Gamepad OP_PAD2 = new Gamepad(2);

  private RavenPiColorSensor _colorSensor = new RavenPiColorSensor();
  
  public static final DriveTrainSubsystemBase DRIVE_TRAIN_SUBSYSTEM = new DriveTrainSubsystem();
  public static final ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem();
  public static final IntakeExtenderSubsystem INTAKE_SUBSYSTEM = new IntakeExtenderSubsystem();
  public static final ConveyanceSubsystem CONVEYANCE_SUBSYSTEM = new ConveyanceSubsystem();
  // public static final IntakeExtendCommand IntakeExtend = new IntakeExtendCommand();
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
  public static final FeederWheelReverseCommand FeederWheelReverse = new FeederWheelReverseCommand();
  public static final ShooterLowGoalCommand SHOOTER_LOW_GOAL_PID_COMMAND = new ShooterLowGoalCommand();
  public static final ShooterTarmacCommand SHOOTER_TARMAC_PID_COMMAND = new ShooterTarmacCommand();
  public static final ShooterLaunchpadCommand SHOOTER_LAUNCH_PAD_PID_COMMAND = new ShooterLaunchpadCommand();
  public static final ShooterAutoRadiusCommand SHOOTER_AUTO_RADIUS_PID_COMMAND = new ShooterAutoRadiusCommand();
  public static final FeederCollectCommand FeederCollect = new FeederCollectCommand();
  public static final ClimberDefaultBrakeCommand climberDefaultBrake = new ClimberDefaultBrakeCommand();
  public static final CompressorSubsystem COMPRESSOR_SUBSYSTEM = new CompressorSubsystem();
  public static final LimelightSubsystem LIMELIGHT_SUBSYSTEM = new LimelightSubsystem();
  public static final TurretAimAtTargetCommand TURRET_AIM_AT_TARGET = new TurretAimAtTargetCommand();
  public static final TurretFlipCommand TURRET_FLIP = new TurretFlipCommand();
  public static final TurretSeekCommand TURRET_SEEK = new TurretSeekCommand();
  public static final DrivetrainDefaultCommand DRIVE_TRAIN_DEFAULT_COMMAND = new DrivetrainDefaultCommand();
  public static final FeederShootOneBallCommand FEEDER_SHOOT_ONE_BALL = new FeederShootOneBallCommand();
  public static final RavenBlinkin RAVEN_BLINKIN_3 = new RavenBlinkin(3);
  public static final RavenBlinkin RAVEN_BLINKIN_4 = new RavenBlinkin(4);
  public static final AutoMode TWO_BALL_HANGAR_AUTO = new AutoMode("Two Ball Hangar", TwoBallAutoCommand.getHangarCommand());
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DRIVE_TRAIN_SUBSYSTEM.setDefaultCommand(DRIVE_TRAIN_DEFAULT_COMMAND);
    //SHOOTER_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> SHOOTER_SUBSYSTEM.defaultCommand(), SHOOTER_SUBSYSTEM));

    SHOOTER_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> SHOOTER_SUBSYSTEM.defaultCommand(), SHOOTER_SUBSYSTEM));
    // TURRET_SWIVEL_SUBSYSTEM.setDefaultCommand(TURRET_AIM_AT_TARGET);
    FEEDER_SUBSYSTEM.setDefaultCommand(FeederIndex);
    CLIMBER_SUBSYSTEM.setDefaultCommand(climberDefaultBrake);
    CONVEYANCE_SUBSYSTEM.setDefaultCommand(CONVEYANCE_INDEX_COMMAND);
    configureButtonBindings();
    LIMELIGHT_SUBSYSTEM.turnLEDOff();
    CameraServer.startAutomaticCapture();

    _autoChooser.setDefaultOption(TWO_BALL_HANGAR_AUTO.getAutoName(), TWO_BALL_HANGAR_AUTO);
    _autoChooser.addOption("Two Ball Wall", new AutoMode("Two Ball Wall", TwoBallAutoCommand.getWallCommand()));
    _autoChooser.addOption("Three Ball Tarmac", new AutoMode("Three Ball Tarmac", ThreeBallTarmacAutoCommand.get()));
    _autoChooser.addOption("Five Ball HPS", new AutoMode("Five Ball HPS", FiveBallHps.get()));
  }

  private AutoMode getAuto() {
    var autoMode = _autoChooser.getSelected();
    if (autoMode == null) {
      return TWO_BALL_HANGAR_AUTO;
    }

    return autoMode;
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
    SmartDashboard.putData("Autonomous", _autoChooser);
    SmartDashboard.putString("Chosen Auto", this.getAuto().getAutoName());
    

    SmartDashboard.putBoolean("Target Sighted", Robot.LIMELIGHT_SUBSYSTEM.hasTargetSighted());
    SmartDashboard.putNumber("Limelight Offset", Robot.LIMELIGHT_SUBSYSTEM.getTargetOffsetAngle());
    SmartDashboard.putNumber("Limelight Area", Robot.LIMELIGHT_SUBSYSTEM.getArea());
    if (GAMEPAD.getAxisIsPressed(AxisCode.LEFTTRIGGER)) {
      Robot.LIMELIGHT_SUBSYSTEM.turnLEDOn();
    } else {
      Robot.LIMELIGHT_SUBSYSTEM.turnLEDOff();
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
    m_autonomousCommand = this.getAuto().getAutoCommand();
    
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
    var shootGarbarge = new Trigger(() -> {
      boolean ballIsRed = _colorSensor.getBallType(RavenPiPosition.EXIT) == RavenPiColor.RED;
      boolean blueAlliance = DriverStation.getAlliance() == Alliance.Blue;
      boolean ballIsBlue = _colorSensor.getBallType(RavenPiPosition.EXIT) == RavenPiColor.BLUE;
      boolean redAlliance = DriverStation.getAlliance() == Alliance.Red;

      return ballIsRed && blueAlliance || ballIsBlue && redAlliance;
    });

    shootGarbarge.whenActive(new JunkShotCommandGroup(), false);

    SHOOTER_SUBSYSTEM.resetShotCount();

    // Stop any autonomous command that might still be running.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if (Robot.LIMELIGHT_SUBSYSTEM.isAligned()) {
      if (Robot.SHOOTER_SUBSYSTEM.getReadyToShootTarmac()) {
        RAVEN_BLINKIN_4.setBlink(BlinkinCalibrations.BLUE);
      }
      else {
        RAVEN_BLINKIN_4.setSolid(BlinkinCalibrations.BLUE);
      }
    }
    else {
      RAVEN_BLINKIN_4.setSolid(BlinkinCalibrations.OFF);
    }


    if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceHasBall() == false && Robot.FEEDER_SUBSYSTEM.getFeederHasBall() == false) {
      RAVEN_BLINKIN_3.setSolid(BlinkinCalibrations.RED);
    }
    else if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceHasBall() == true && Robot.FEEDER_SUBSYSTEM.getFeederHasBall() == true ){
      if (SHOOTER_SUBSYSTEM.motorsAreSpinning()) {
        RAVEN_BLINKIN_3.setBlink(BlinkinCalibrations.GREEN);
      }
      else {
        RAVEN_BLINKIN_3.setSolid(BlinkinCalibrations.GREEN);
      }
    }
    else if(Robot.CONVEYANCE_SUBSYSTEM.getConveyanceHasBall() == true || Robot.FEEDER_SUBSYSTEM.getFeederHasBall() == true) {
      if (SHOOTER_SUBSYSTEM.motorsAreSpinning()) {
        RAVEN_BLINKIN_3.setBlink(BlinkinCalibrations.YELLOW);
      }
      else {
        RAVEN_BLINKIN_3.setSolid(BlinkinCalibrations.YELLOW);
      }      
    }
  }
  
  public void configureButtonBindings() {
    
    GAMEPAD.getButton(ButtonCode.LEFTBUMPER)
      .and(GAMEPAD.getButton(ButtonCode.RIGHTBUMPER))
      .and(GAMEPAD.getButton(ButtonCode.Y))
      .whenActive(new InstantCommand(DRIVE_TRAIN_SUBSYSTEM::zeroGyroscope, DRIVE_TRAIN_SUBSYSTEM));
    

    OP_PAD.getButton(ButtonCode.CLIMBER_OVERRIDE)
      .whenActive(new InstantCommand(CLIMBER_SUBSYSTEM::turnOverrideOn))
      .whenInactive(new InstantCommand(CLIMBER_SUBSYSTEM::turnOverrideOff));

    new Trigger(() -> GAMEPAD.getAxisIsPressed(AxisCode.RIGHTTRIGGER))
      .whenActive(DRIVE_TRAIN_SUBSYSTEM::cutPower)
      .whenInactive(DRIVE_TRAIN_SUBSYSTEM::stopCutPower);

    new Trigger(() -> GAMEPAD.getAxisIsPressed(AxisCode.LEFTTRIGGER))
      .whenActive(() -> DRIVE_TRAIN_DEFAULT_COMMAND.followLimelight())
      .whenInactive(() -> DRIVE_TRAIN_DEFAULT_COMMAND.stopFollowingLimelight());

    
    OP_PAD2.getButton(ButtonCode.CLIMBER_RETRACT).whileHeld(new StartEndCommand(
      () -> Robot.CLIMBER_SUBSYSTEM.retract(),
      () -> Robot.CLIMBER_SUBSYSTEM.stop(),
      Robot.CLIMBER_SUBSYSTEM
    ));
    OP_PAD2.getButton(ButtonCode.CLIMBER_EXTEND).whileHeld(new StartEndCommand(
      () -> Robot.CLIMBER_SUBSYSTEM.extend(),
      () -> Robot.CLIMBER_SUBSYSTEM.stop(),
      Robot.CLIMBER_SUBSYSTEM
    ));
    OP_PAD2.getButton(ButtonCode.CLIMBER_RETRACT_SLOWLY).whileHeld(new StartEndCommand(
      () -> Robot.CLIMBER_SUBSYSTEM.retractSlowly(),
      () -> Robot.CLIMBER_SUBSYSTEM.stop(),
      Robot.CLIMBER_SUBSYSTEM
    ));
    OP_PAD2.getButton(ButtonCode.CLIMBER_EXTEND_SLOWLY).whileHeld(new StartEndCommand(
      () -> Robot.CLIMBER_SUBSYSTEM.extendSlowly(),
      () -> Robot.CLIMBER_SUBSYSTEM.stop(),
      Robot.CLIMBER_SUBSYSTEM
    ));
    
    OP_PAD.getButton(ButtonCode.SHOOTER_REV)
      .whileHeld(SHOOTER_START_COMMAND)
      .whenInactive(SHOOTER_STOP_COMMAND);

    GAMEPAD.getButton(ButtonCode.RIGHTBUMPER).whileHeld(CONVEYANCE_COLLECT_COMMAND);
    //GAMEPAD.getButton(ButtonCode.B).whileHeld(new SequentialCommandGroup(new WaitCommand(.15), SHOOTER_START_COMMAND));
    GAMEPAD.getButton(ButtonCode.LEFTBUMPER).whileHeld(CONVEYANCE_EJECT_COMMAND);
    //GAMEPAD.getButton(ButtonCode.B).whenPressed(FeederSafetyReverse);
    GAMEPAD.getButton(ButtonCode.A).whileHeld(FeederShoot);
    OP_PAD2.getButton(ButtonCode.FEEDER_WHEEL_REVERSE).whileHeld(FeederWheelReverse);
    //GAMEPAD.getButton(ButtonCode.BACK).whenHeld(TURRET_FLIP);
    //GAMEPAD.getButton(ButtonCode.START).whenHeld(TURRET_SEEK);
    // GAMEPAD.getButton(ButtonCode.A).whileHeld(FeederCollect);
    GAMEPAD.getButton(ButtonCode.B).whileHeld(FeederEject);
    //GAMEPAD.getButton(ButtonCode.Y).whenPressed(FeederSafetyReverse);
    // GAMEPAD.getButton(ButtonCode.Y).whenPressed(FEEDER_SHOOT_ONE_BALL);
    OP_PAD.getButton(ButtonCode.SHOOTER_LAUNCH_PAD_SHOT).whenPressed(SHOOTER_LAUNCH_PAD_PID_COMMAND);
    OP_PAD.getButton(ButtonCode.SHOOTER_TARMAC_SHOT).whenPressed(SHOOTER_TARMAC_PID_COMMAND);
    OP_PAD.getButton(ButtonCode.SHOOTER_LOW_GOAL_SHOT).whenPressed(SHOOTER_LOW_GOAL_PID_COMMAND);
    OP_PAD.getButton(ButtonCode.SHOOTER_AUTO_RADIUS_SHOT).whenPressed(SHOOTER_AUTO_RADIUS_PID_COMMAND);
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
