// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
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
import frc.ravenhardware.RavenPiColorSensor;
import frc.ravenhardware.RavenPiPosition;
import frc.robot.commands.auto.FiveBallHps;
import frc.robot.commands.auto.ThreeBallTarmacAutoCommand;
import frc.robot.commands.auto.TwoBallAutoCommand;
import frc.robot.commands.climber.ClimberDefaultBrakeCommand;
import frc.robot.commands.climber.ClimberExtendCommand;
import frc.robot.commands.climber.ClimberRetractCommand;
import frc.robot.commands.commandgroups.ConveyanceFeederEjectAllCommand;
import frc.robot.commands.commandgroups.JunkShotCommandGroup;
import frc.robot.commands.conveyance.ConveyanceCollectCommand;
import frc.robot.commands.conveyance.ConveyanceEjectCommand;
import frc.robot.commands.conveyance.ConveyanceIndexCommand;
import frc.robot.commands.conveyance.IntakeRetractCommand;
import frc.robot.commands.drivetrain.DrivetrainDefaultCommand;
import frc.robot.commands.feeder.FeederCollectCommand;
import frc.robot.commands.feeder.FeederIndexCommand;
import frc.robot.commands.feeder.FeederSafetyReverseCommand;
import frc.robot.commands.feeder.FeederShootCommand;
import frc.robot.commands.feeder.FeederShootOneBallCommand;
import frc.robot.commands.feeder.FeederUnloadCommand;
import frc.robot.commands.feeder.FeederWheelReverseCommand;
import frc.robot.commands.shooter.ShooterAutoRadiusCommand;
import frc.robot.commands.shooter.ShooterLaunchpadCommand;
import frc.robot.commands.shooter.ShooterLowGoalCommand;
import frc.robot.commands.shooter.ShooterStartCommand;
import frc.robot.commands.shooter.ShooterStopCommand;
import frc.robot.commands.shooter.ShooterTarmacCommand;
import frc.robot.commands.turret.TurretAimAtTargetCommand;
import frc.robot.commands.turret.TurretFlipCommand;
import frc.robot.commands.turret.TurretHomeCommand;
import frc.robot.commands.turret.TurretSeekCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.ConveyanceSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystemBase;
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
  public static final Gamepad OP_PAD = new Gamepad(1);
  public static final Gamepad OP_PAD2 = new Gamepad(2);
  
  public static final DrivetrainSubsystemBase DRIVE_TRAIN_SUBSYSTEM = new DrivetrainSubsystem();
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
  public static final ClimberExtendCommand CLIMBER_EXTEND_COMMAND = new ClimberExtendCommand();
  public static final ClimberRetractCommand CLIMBER_RETRACT_COMMAND = new ClimberRetractCommand();
  public static final CompressorSubsystem COMPRESSOR_SUBSYSTEM = new CompressorSubsystem();
  public static final LimelightSubsystem LIMELIGHT_SUBSYSTEM = new LimelightSubsystem();
  public static final TurretAimAtTargetCommand TURRET_AIM_AT_TARGET = new TurretAimAtTargetCommand();
  public static final TurretFlipCommand TURRET_FLIP = new TurretFlipCommand();
  public static final TurretSeekCommand TURRET_SEEK = new TurretSeekCommand();
  public static final DrivetrainDefaultCommand DRIVE_TRAIN_DEFAULT_COMMAND = new DrivetrainDefaultCommand();
  public static final FeederShootOneBallCommand FEEDER_SHOOT_ONE_BALL = new FeederShootOneBallCommand();
  public static final RavenBlinkin RAVEN_BLINKIN_3 = new RavenBlinkin(3);
  public static final RavenBlinkin RAVEN_BLINKIN_4 = new RavenBlinkin(4);
  public static final AutoMode TWO_BALL_HANGAR_AUTO = TwoBallAutoCommand.getHangarAutoMode();
  public static final ConveyanceFeederEjectAllCommand CONVEYANCE_FEEDER_EJECT_ALL_COMMAND = new ConveyanceFeederEjectAllCommand();
  public static final TurretHomeCommand TURRET_HOME_COMMAND = new TurretHomeCommand();
  public static final FeederUnloadCommand FEEDER_UNLOAD_COMMAND = new FeederUnloadCommand();

  public static final RavenPiColorSensor COLOR_SENSOR = new RavenPiColorSensor();
  public static Alliance ALLIANCE_COLOR;
  public static boolean autonomousTriggerOverride = true;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ALLIANCE_COLOR = DriverStation.getAlliance();

    DRIVE_TRAIN_SUBSYSTEM.setDefaultCommand(DRIVE_TRAIN_DEFAULT_COMMAND);
    //SHOOTER_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> SHOOTER_SUBSYSTEM.defaultCommand(), SHOOTER_SUBSYSTEM));

    SHOOTER_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> SHOOTER_SUBSYSTEM.defaultCommand(), SHOOTER_SUBSYSTEM));
//    TURRET_SWIVEL_SUBSYSTEM.setDefaultCommand(Constants.TURRET_ENABLED ? TURRET_AIM_AT_TARGET : new InstantCommand());
TURRET_SWIVEL_SUBSYSTEM.setDefaultCommand(TURRET_AIM_AT_TARGET);
    
FEEDER_SUBSYSTEM.setDefaultCommand(FeederIndex);
    CLIMBER_SUBSYSTEM.setDefaultCommand(climberDefaultBrake);
    CONVEYANCE_SUBSYSTEM.setDefaultCommand(CONVEYANCE_INDEX_COMMAND);
    configureButtonBindings();
    LIMELIGHT_SUBSYSTEM.turnLEDOff();
    CameraServer.startAutomaticCapture();

    AutoMode twoBallWall = TwoBallAutoCommand.getWallAutoMode();
    AutoMode threeBallTarmac = ThreeBallTarmacAutoCommand.getAutoMode(twoBallWall.getAutoCommand());
    AutoMode fiveBallHps = FiveBallHps.getAutoMode(threeBallTarmac.getAutoCommand());
    AutoMode twoBallHangarPlusHangar = TwoBallAutoCommand.getHangarPlusOtherBallsHangarAutoMode(TWO_BALL_HANGAR_AUTO.getAutoCommand());
    AutoMode twoBallHangarPlusGoal = TwoBallAutoCommand.getHangarPlusOtherBallsByGoalAutoMode(TWO_BALL_HANGAR_AUTO.getAutoCommand());

    TWO_BALL_HANGAR_AUTO.setDefaultOption(_autoChooser);
    twoBallWall.addOption(_autoChooser);
    threeBallTarmac.addOption(_autoChooser);
    fiveBallHps.addOption(_autoChooser);
    twoBallHangarPlusHangar.addOption(_autoChooser);
    twoBallHangarPlusGoal.addOption(_autoChooser);
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

    triggerDashboardPeriodic();
    

    // SmartDashboard.putBoolean("Target Sighted", Robot.LIMELIGHT_SUBSYSTEM.hasTargetSighted());
    // SmartDashboard.putNumber("Limelight Offset", Robot.LIMELIGHT_SUBSYSTEM.getRawTargetOffsetAngle());
    // SmartDashboard.putNumber("Limelight Area", Robot.LIMELIGHT_SUBSYSTEM.getArea());
    
    if (Constants.TURRET_ENABLED || GAMEPAD.getAxisIsPressed(AxisCode.LEFTTRIGGER)) {
      Robot.LIMELIGHT_SUBSYSTEM.turnLEDOn();
    } else {
      Robot.LIMELIGHT_SUBSYSTEM.turnLEDOff();
    }

    SmartDashboard.putString("CONVEYANCE COLOR", Robot.COLOR_SENSOR.getSensorBallColor(RavenPiPosition.CONVEYANCE).toString());
    SmartDashboard.putString("FEEDER COLOR", Robot.COLOR_SENSOR.getSensorBallColor(RavenPiPosition.FEEDER).toString());
    // SmartDashboard.putBoolean("FEEDER HAS CORRECT BALL", Robot.COLOR_SENSOR.getSensorIsCorrectBallColorStrict(RavenPiPosition.FEEDER))
  
    int sensor0Green = Robot.COLOR_SENSOR.getRawColor0().green;
    int sensor0Red = Robot.COLOR_SENSOR.getRawColor0().red;
    int sensor0Blue = Robot.COLOR_SENSOR.getRawColor0().blue;

    int sensor1Green = Robot.COLOR_SENSOR.getRawColor1().green;
    int sensor1Red = Robot.COLOR_SENSOR.getRawColor1().red;
    int sensor1Blue = Robot.COLOR_SENSOR.getRawColor1().blue;

    SmartDashboard.putNumber("CSENSOR 0 red", sensor0Red);
    SmartDashboard.putNumber("CSENSOR 0 Green", sensor0Green);
    SmartDashboard.putNumber("CSENSOR 0 blue", sensor0Blue);
    
    SmartDashboard.putNumber("CSENSOR 1 red", sensor1Red);
    SmartDashboard.putNumber("CSENSOR 1 Green", sensor1Green);
    SmartDashboard.putNumber("CSENSOR 1 blue", sensor1Blue);

  }
   /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousTriggerOverride = true;

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
    autonomousTriggerOverride = false;

    SHOOTER_SUBSYSTEM.resetShotCount();
    COLOR_SENSOR.setColorSensorFeatureEnabled(true);

    // Stop any autonomous command that might still be running.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  public void triggerDashboardPeriodic() {
    SmartDashboard.putBoolean("RobotHas2Balls", CommonTriggers.RobotHas2Balls.getAsBoolean());
    SmartDashboard.putBoolean("RunShooterTrigger", CommonTriggers.RunShooterTrigger.getAsBoolean());
    SmartDashboard.putBoolean("RunAutoshootingTrigger", CommonTriggers.RunAutoshootingTrigger.getAsBoolean());
    SmartDashboard.putBoolean("ReleaseBallTrigger", CommonTriggers.ReleaseBallTrigger.getAsBoolean());
    SmartDashboard.putNumber("Cargo Inventory", getRobotCargoInventory());
    SmartDashboard.putString("Auto shot select", Robot.SHOOTER_SUBSYSTEM.getShot()._name);

  
    boolean hasAmmo = (Robot.getRobotProperColorInventory() >= 2 || Robot.GAMEPAD.getAxisIsPressed(AxisCode.LEFTTRIGGER)); 


    SmartDashboard.putBoolean("LIMELIGHT ALIGNED", Robot.LIMELIGHT_SUBSYSTEM.isAligned());
    SmartDashboard.putBoolean("RPM GOOD", Robot.SHOOTER_SUBSYSTEM.motorsAreRecovered());
    SmartDashboard.putBoolean("AUTOSHOOT ACTIVE", Robot.SHOOTER_SUBSYSTEM.getAutoShotSelect());
    SmartDashboard.putBoolean("HAS AMMO", hasAmmo);

    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    /*
    if (Timer.getFPGATimestamp() <= 60) {
      RAVEN_BLINKIN_3.blinkGreen();
    } else if (Timer.getFPGATimestamp() <= 30) {
      RAVEN_BLINKIN_3.blinkYellow();
    } else if (Timer.getFPGATimestamp() <= 15) {
      RAVEN_BLINKIN_3.blinkRed();
    }
    */
/*
    if (SHOOTER_SUBSYSTEM.motorsAreRecovered()) {
      RAVEN_BLINKIN_3.solidGreen();
      // RAVEN_BLINKIN_3.blinkGreen();
    }
    */

    if (Robot.LIMELIGHT_SUBSYSTEM.isAligned()) {
      if (Robot.SHOOTER_SUBSYSTEM.getReadyToShootTarmac()) {
        RAVEN_BLINKIN_4.setBlink(BlinkinCalibrations.BLUE);
      }
      else {
        RAVEN_BLINKIN_4.setSolid(BlinkinCalibrations.BLUE);
      }
    }
    else if (Robot.LIMELIGHT_SUBSYSTEM.hasTargetSighted()) {
      RAVEN_BLINKIN_4.setSolid(BlinkinCalibrations.YELLOW);
    }
    else {
      RAVEN_BLINKIN_4.setSolid(BlinkinCalibrations.RED);
    }


    if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceStagingBeamBreakHasBall() == false && Robot.FEEDER_SUBSYSTEM.getFeederHasBall() == false) {
      RAVEN_BLINKIN_3.setSolid(BlinkinCalibrations.RED);
    }
    else if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceStagingBeamBreakHasBall() == true && Robot.FEEDER_SUBSYSTEM.getFeederHasBall() == true ){
      if (SHOOTER_SUBSYSTEM.motorsAreSpinning()) {
        RAVEN_BLINKIN_3.setBlink(BlinkinCalibrations.GREEN);
      }
      else {
        RAVEN_BLINKIN_3.setSolid(BlinkinCalibrations.GREEN);
      }
    }
    else if(Robot.CONVEYANCE_SUBSYSTEM.getConveyanceStagingBeamBreakHasBall() == true || Robot.FEEDER_SUBSYSTEM.getFeederHasBall() == true) {
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
      .whenActive(DRIVE_TRAIN_SUBSYSTEM::zeroGyroscope);
    

    OP_PAD.getButton(ButtonCode.CLIMBER_OVERRIDE)
      .whileHeld(new InstantCommand(CLIMBER_SUBSYSTEM::turnOverrideOn))
      .whenInactive(new InstantCommand(CLIMBER_SUBSYSTEM::turnOverrideOff));

    Trigger shootGarbarge = new Trigger(() -> {
      if (Robot.autonomousTriggerOverride == true) {
          return false;
      }
      
      return FEEDER_SUBSYSTEM.feederHasWrongColorCargo();
    });

    shootGarbarge.whileActiveContinuous(new JunkShotCommandGroup(), false);

    new Trigger(() -> GAMEPAD.getAxisIsPressed(AxisCode.RIGHTTRIGGER) || Robot.CLIMBER_SUBSYSTEM.climberIsExtended())
      .whileActiveContinuous(DRIVE_TRAIN_SUBSYSTEM::cutPower)
      .whenInactive(DRIVE_TRAIN_SUBSYSTEM::stopCutPower);

    // new Trigger(() -> GAMEPAD.getAxisIsPressed(AxisCode.LEFTTRIGGER)).or(CommonTriggers.RobotHas2Balls)
    if (Constants.TURRET_ENABLED == false) {
      CommonTriggers.RunAutoshootingTrigger
        .whileActiveContinuous(() -> DRIVE_TRAIN_DEFAULT_COMMAND.followLimelight())
        .whenInactive(() -> DRIVE_TRAIN_DEFAULT_COMMAND.stopFollowingLimelight());
    }

    CommonTriggers.RunShooterTrigger
      .whileActiveContinuous(SHOOTER_START_COMMAND)
      // .whenActive(SHOOTER_START_COMMAND)
      .whenInactive(SHOOTER_STOP_COMMAND);

    CommonTriggers.ReleaseBallTrigger
      .whenActive(FEEDER_UNLOAD_COMMAND);
//      .whileActiveContinuous(FeederShoot);

      /*
      Old shooter activation code, pending deletion if new triggers work
    OP_PAD.getButton(ButtonCode.SHOOTER_REV)
      .whileHeld(SHOOTER_START_COMMAND)
      .whenInactive(SHOOTER_STOP_COMMAND);
*/

    CommonTriggers.RobotHas2Balls.negate().and(GAMEPAD.getButton(ButtonCode.RIGHTBUMPER))
      .whileActiveOnce(CONVEYANCE_COLLECT_COMMAND);

    OP_PAD2.getButton(ButtonCode.CLIMBER_EXTEND).whileHeld(CLIMBER_EXTEND_COMMAND);
    OP_PAD2.getButton(ButtonCode.CLIMBER_RETRACT).whileHeld(CLIMBER_RETRACT_COMMAND);
/*
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

    */
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

    OP_PAD.getButton(ButtonCode.SHOOTER_PROFILE_MANUAL_OVERRIDE)
      .whileHeld(() -> Robot.SHOOTER_SUBSYSTEM.disableAutoShotSelect())
      .whenInactive(() -> Robot.SHOOTER_SUBSYSTEM.enableAutoShotSelect());
      
    GAMEPAD.getButton(ButtonCode.LEFTBUMPER).or(CommonTriggers.AutosteerTrigger)
      .whileActiveContinuous(DRIVE_TRAIN_DEFAULT_COMMAND::disableAutoSteer)
      .whenInactive(DRIVE_TRAIN_DEFAULT_COMMAND::enableAutoSteer);

    GAMEPAD.getButton(ButtonCode.A).whileHeld(FeederShoot);
    OP_PAD2.getButton(ButtonCode.FEEDER_WHEEL_REVERSE).whileHeld(FeederWheelReverse);
    // GAMEPAD.getButton(ButtonCode.B).whileHeld(new ParallelCommandGroup(new FeederEjectAllCommand(), CONVEYANCE_EJECT_COMMAND));
    //GAMEPAD.getButton(ButtonCode.B).whileHeld(new FeederEjectAllCommand());
    
    GAMEPAD.getButton(ButtonCode.B).whileHeld(CONVEYANCE_FEEDER_EJECT_ALL_COMMAND);
    // GAMEPAD.getButton(ButtonCode.B).whileHeld(CONVEYANCE_EJECT_COMMAND);
    OP_PAD.getButton(ButtonCode.SHOOTER_LAUNCH_PAD_SHOT).whenPressed(SHOOTER_LAUNCH_PAD_PID_COMMAND);
    OP_PAD.getButton(ButtonCode.SHOOTER_TARMAC_SHOT).whenPressed(SHOOTER_TARMAC_PID_COMMAND);
    OP_PAD.getButton(ButtonCode.SHOOTER_LOW_GOAL_SHOT).whenPressed(SHOOTER_LOW_GOAL_PID_COMMAND);
    OP_PAD.getButton(ButtonCode.SHOOTER_AUTO_RADIUS_SHOT).whenPressed(SHOOTER_AUTO_RADIUS_PID_COMMAND);
    OP_PAD2.getButton(ButtonCode.TURRET_HOME).whenPressed(TURRET_HOME_COMMAND.withTimeout(1));
    // Old assignments, pending deletion
    //GAMEPAD.getButton(ButtonCode.B).whileHeld(new SequentialCommandGroup(new WaitCommand(.15), SHOOTER_START_COMMAND));
    //GAMEPAD.getButton(ButtonCode.B).whenPressed(FeederSafetyReverse);
    //GAMEPAD.getButton(ButtonCode.BACK).whenHeld(TURRET_FLIP);
    //GAMEPAD.getButton(ButtonCode.START).whenHeld(TURRET_SEEK);
    // GAMEPAD.getButton(ButtonCode.A).whileHeld(FeederCollect);
    //GAMEPAD.getButton(ButtonCode.Y).whenPressed(FeederSafetyReverse);
    // GAMEPAD.getButton(ButtonCode.Y).whenPressed(FEEDER_SHOOT_ONE_BALL);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /*
  public boolean giveLimelightDriveControl() {
    boolean giveControl = false;

    boolean leftTrigger = GAMEPAD.getAxisIsPressed(AxisCode.LEFTTRIGGER);

    boolean ammoFull;


    return giveControl;
  }
  */

  public static int getRobotCargoInventory() {
    int inventory = 0;

    if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceEntryBeamBreakHasBall() || Robot.CONVEYANCE_SUBSYSTEM.getIsIndexingFromEntranceToStaging()) {
      inventory++;
    }
    
    if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceStagingBeamBreakHasBall() || (Robot.CONVEYANCE_SUBSYSTEM.getIsIndexingFromStagingToFeeder() && Robot.CONVEYANCE_SUBSYSTEM.getConveyanceEntryBeamBreakHasBall() == false)) {
      inventory++;
    }

    if (Robot.FEEDER_SUBSYSTEM.getFeederHasBall()) {
      inventory++;
    }

    return inventory;
  }

  // Unlike the general get inventory method, this method does not keep track of indexing balls.
  public static int getRobotProperColorInventory() {
    int inventory = 0;

    if (Robot.CONVEYANCE_SUBSYSTEM.conveyanceHasProperColorCargo()) {
      inventory++;
    }

    if (Robot.FEEDER_SUBSYSTEM.feederHasProperColorCargo()) {
      inventory++;
    }
    
    return inventory;
  }

}
