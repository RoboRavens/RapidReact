// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.RavenSwerveControllerCommand;
import frc.util.Deadband;
import frc.util.DriveCharacteristics;
import frc.util.SwerveModuleConverter;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static frc.robot.RobotMap.*;

import java.util.List;

// Template From: https://github.com/SwerveDriveSpecialties/swerve-template/blob/master/src/main/java/frc/robot/subsystems/DrivetrainSubsystem.java
public class DrivetrainSubsystem extends DrivetrainSubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4_L1.getDriveReduction() *
          SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0), // Front right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0) // Back right
  );
  
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);

  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;
  private final SwerveDriveOdometry _odometryFromKinematics;
  private final SwerveDriveOdometry  _odometryFromHardware;

  // private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private SwerveModuleState[] _moduleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0,0,0));

  private final NetworkTableEntry _nteMaxVelocity;
  private final NetworkTableEntry _nteMaxAngular;

  private final NetworkTableEntry _odometryXKinematics;
  private final NetworkTableEntry _odometryYKinematics;
  private final NetworkTableEntry _odometryAngleKinematics;
  private final NetworkTableEntry _frontLeftKinematics;
  private final NetworkTableEntry _frontRightKinematics;
  private final NetworkTableEntry _backLeftKinematics;
  private final NetworkTableEntry _backRightKinematics;
  private final NetworkTableEntry _frontLeftKinematicsVel;

  private final NetworkTableEntry _odometryXHardware;
  private final NetworkTableEntry _odometryYHardware;
  private final NetworkTableEntry _odometryAngleHardware;
  private final NetworkTableEntry _frontLeftHardware;
  private final NetworkTableEntry _frontRightHardware;
  private final NetworkTableEntry _backLeftHardware;
  private final NetworkTableEntry _backRightHardware;
  private final NetworkTableEntry _frontLeftHardwareVel;

  private final DriveCharacteristics _driveCharacteristics;

  private Boolean _cutPower = false;
  private Pose2d _markedPosition = null;

  public DrivetrainSubsystem() {
    m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            Mk4SwerveModuleHelper.GearRatio.L1,
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            FRONT_LEFT_MODULE_STEER_MOTOR,
            FRONT_LEFT_MODULE_STEER_ENCODER,
            FRONT_LEFT_MODULE_STEER_OFFSET // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
    );

    // We will do the same for the other modules
    m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
        Mk4SwerveModuleHelper.GearRatio.L1,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
        Mk4SwerveModuleHelper.GearRatio.L1,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
        Mk4SwerveModuleHelper.GearRatio.L1,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );

    _odometryFromKinematics = new SwerveDriveOdometry(m_kinematics, this.getGyroscopeRotation(), new Pose2d(0, 0, new Rotation2d()));
    _odometryFromHardware = new SwerveDriveOdometry(m_kinematics, this.getGyroscopeRotation(), new Pose2d(0, 0, new Rotation2d()));

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    _nteMaxVelocity  = tab.add("Max Velocity", 0.0).withPosition(0, 0).withSize(1, 1).getEntry();
    _nteMaxAngular  = tab.add("Max Angular Vel", 0.0).withPosition(0, 1).withSize(1, 1).getEntry();

    _nteMaxVelocity.setDouble(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND);
    _nteMaxAngular.setDouble(DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

    _odometryXKinematics = tab.add("X Kinematics", 0.0).withPosition(8, 0).withSize(1, 1).getEntry();
    _odometryYKinematics = tab.add("Y Kinematics", 0.0).withPosition(8, 1).withSize(1, 1).getEntry();
    _odometryAngleKinematics = tab.add("Angle Kinematics", 0.0).withPosition(8, 2).withSize(1, 1).getEntry();
    _frontLeftKinematics = tab.add("FL Kinematics A", 0.0).withPosition(4, 0).withSize(1, 1).getEntry();
    _frontRightKinematics  = tab.add("FR Kinematics A", 0.0).withPosition(4, 1).withSize(1, 1).getEntry();
    _backLeftKinematics  = tab.add("BL Kinematics A", 0.0).withPosition(4, 2).withSize(1, 1).getEntry();
    _backRightKinematics  = tab.add("BR Kinematics A", 0.0).withPosition(4, 3).withSize(1, 1).getEntry();
    _frontLeftKinematicsVel = tab.add("FL Kinematics V", 0.0).withPosition(3, 0).withSize(1, 1).getEntry();

    _odometryXHardware = tab.add("X Hardware", 0.0).withPosition(9, 0).withSize(1, 1).getEntry();
    _odometryYHardware = tab.add("Y Hardware", 0.0).withPosition(9, 1).withSize(1, 1).getEntry();
    _odometryAngleHardware = tab.add("Angle Hardware", 0.0).withPosition(9, 2).withSize(1, 1).getEntry();
    _frontLeftHardware = tab.add("FL Hardware A", 0.0).withPosition(5, 0).withSize(1, 1).getEntry();
    _frontRightHardware  = tab.add("FR Hardware A", 0.0).withPosition(5, 1).withSize(1, 1).getEntry();
    _backLeftHardware  = tab.add("BL Hardware A", 0.0).withPosition(5, 2).withSize(1, 1).getEntry();
    _backRightHardware  = tab.add("BR Hardware A", 0.0).withPosition(5, 3).withSize(1, 1).getEntry();
    _frontLeftHardwareVel = tab.add("FL Hardware V", 0.0).withPosition(6, 0).withSize(1, 1).getEntry();

    _driveCharacteristics = new DriveCharacteristics();
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  @Override
  public void zeroGyroscope() {
    m_navx.zeroYaw();
    _odometryFromKinematics.resetPosition(new Pose2d(0, 0, new Rotation2d()), this.getGyroscopeRotation());
    _odometryFromHardware.resetPosition(new Pose2d(0, 0, new Rotation2d()), this.getGyroscopeRotation());
    _driveCharacteristics.reset();
  }

  @Override
  public Rotation2d getOdometryRotation() {
    return _odometryFromHardware.getPoseMeters().getRotation();
  }

  private Rotation2d getGyroscopeRotation() {
    if (m_navx.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    }

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    return Rotation2d.fromDegrees(360.0 - m_navx.getAngle());
  }

  @Override
  public void holdPosition() {
    // create an X pattern with the wheels to thwart pushing from other robots
    _moduleStates[0].angle = Rotation2d.fromDegrees(45); // front left
    _moduleStates[0].speedMetersPerSecond = 0;
    _moduleStates[1].angle = Rotation2d.fromDegrees(-45); // front right
    _moduleStates[1].speedMetersPerSecond = 0;
    _moduleStates[2].angle = Rotation2d.fromDegrees(-45); // back left
    _moduleStates[2].speedMetersPerSecond = 0;
    _moduleStates[3].angle = Rotation2d.fromDegrees(45); // back right
    _moduleStates[3].speedMetersPerSecond = 0;
  }

  @Override
  public void cutPower() {
    _cutPower = true;
  }

  @Override
  public void stopCutPower() {
    _cutPower = false;
  }

  @Override
  public boolean powerIsCut() {
    return _cutPower;
  }

  @Override
  public void drive(ChassisSpeeds chassisSpeeds) {
    if (_cutPower) {
      chassisSpeeds.omegaRadiansPerSecond =  chassisSpeeds.omegaRadiansPerSecond * 0.5;
      chassisSpeeds.vxMetersPerSecond =  chassisSpeeds.vxMetersPerSecond * 0.5;
      chassisSpeeds.vyMetersPerSecond =  chassisSpeeds.vyMetersPerSecond * 0.5;
    }

    _moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = _moduleStates; // states and _modulestates still point to the same data
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    _odometryFromKinematics.update(this.getGyroscopeRotation(), states);
    _odometryXKinematics.setDouble(_odometryFromKinematics.getPoseMeters().getX());
    _odometryYKinematics.setDouble(_odometryFromKinematics.getPoseMeters().getY());
    _odometryAngleKinematics.setDouble(_odometryFromKinematics.getPoseMeters().getRotation().getDegrees());
    _frontLeftKinematics.setDouble(states[0].angle.getDegrees());
    _frontLeftKinematicsVel.setDouble(states[0].speedMetersPerSecond);
    _frontRightKinematics.setDouble(states[1].angle.getDegrees());
    _backLeftKinematics.setDouble(states[2].angle.getDegrees());
    _backRightKinematics.setDouble(states[3].angle.getDegrees());

    var statesHardware = new SwerveModuleState[4];
    statesHardware[0] = SwerveModuleConverter.ToSwerveModuleState(m_frontLeftModule, 0);
    statesHardware[1] = SwerveModuleConverter.ToSwerveModuleState(m_frontRightModule, 0);
    statesHardware[2] = SwerveModuleConverter.ToSwerveModuleState(m_backLeftModule, 0);
    statesHardware[3] = SwerveModuleConverter.ToSwerveModuleState(m_backRightModule, 0);
    _odometryFromHardware.update(this.getGyroscopeRotation(), statesHardware);
    _odometryXHardware.setDouble(_odometryFromHardware.getPoseMeters().getX());
    _odometryYHardware.setDouble(_odometryFromHardware.getPoseMeters().getY());
    _odometryAngleHardware.setDouble(_odometryFromHardware.getPoseMeters().getRotation().getDegrees());
    _frontLeftHardware.setDouble(statesHardware[0].angle.getDegrees());
    _frontLeftHardwareVel.setDouble(statesHardware[0].speedMetersPerSecond);
    _frontRightHardware.setDouble(statesHardware[1].angle.getDegrees());
    _backLeftHardware.setDouble(statesHardware[2].angle.getDegrees());
    _backRightHardware.setDouble(statesHardware[3].angle.getDegrees());

    Deadband.adjustRotationWhenStopped(states, statesHardware);
    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());

    this.calculateDriveCharacteristics();
  }

  private Pose2d getPose() {
    return _odometryFromHardware.getPoseMeters();
  }

  private void resetOdometry(Pose2d pose, Rotation2d rotation) {
    var targetPose = new Pose2d(pose.getTranslation(), pose.getRotation());
    _odometryFromKinematics.resetPosition(targetPose, this.getGyroscopeRotation());
    _odometryFromHardware.resetPosition(targetPose, this.getGyroscopeRotation());
  }

  private void setModuleStates(SwerveModuleState[] moduleStates) {
    _moduleStates = moduleStates;
  }

  private void stop() {
    this.drive(new ChassisSpeeds(0,0,0));
  }

  @Override
  public TrajectoryConfig GetTrajectoryConfig() {
    // Create config for trajectory
    TrajectoryConfig config =
    new TrajectoryConfig(
      Constants.TRAJECTORY_CONFIG_MAX_VELOCITY_METERS_PER_SECOND,
      Constants.TRAJECTORY_CONFIG_MAX_ACCELERATION_METERS_PER_SECOND)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(m_kinematics);

    return config;
  }

  @Override
  public Command CreateSetOdometryToTrajectoryInitialPositionCommand(Trajectory trajectory) {
    return new InstantCommand(() -> this.resetOdometry(trajectory.getInitialPose(), trajectory.getInitialPose().getRotation()));
  }

  @Override
  public Command CreateFollowTrajectoryCommand(Trajectory trajectory) {
    return CreateFollowTrajectoryCommand(trajectory, false);
  }

  @Override
  public Command CreateFollowTrajectoryCommandSwerveOptimized(Trajectory trajectory) {
    return CreateFollowTrajectoryCommand(trajectory, true);
  }
  
  private Command CreateFollowTrajectoryCommand(Trajectory trajectory, boolean swerveOptimized) {
    var robotAngleController =
        new ProfiledPIDController(
          Constants.SWERVE_CONTROLLER_ANGLE_KP, 0, 0, Constants.SWERVE_CONTROLLER_ANGULAR_CONSTRAINTS);
    robotAngleController.enableContinuousInput(-Math.PI, Math.PI);

    Command followTrajectory;
    if (swerveOptimized) {
      followTrajectory = CreateSwerveCommandWhichImmediatelyRotatesToRotationOfLastPointInTrajectory(trajectory, robotAngleController);
    } else {
      followTrajectory = CreateSwerveCommandWhichRespectsTheRotationOfEachPoint(trajectory, robotAngleController);
    }

    return followTrajectory
      .andThen(this::stop);
  }

  private RavenSwerveControllerCommand CreateSwerveCommandWhichRespectsTheRotationOfEachPoint(Trajectory trajectory, ProfiledPIDController robotAngleController) {
    return new RavenSwerveControllerCommand(
      trajectory,
      this::getPose, // Functional interface to feed supplier
      m_kinematics,

      // Position controllers
      new PIDController(Constants.SWERVE_CONTROLLER_X_KP, 0, 0),
      new PIDController(Constants.SWERVE_CONTROLLER_Y_KP, 0, 0),
      robotAngleController,
      this::setModuleStates,
      this);
  }

  private SwerveControllerCommand CreateSwerveCommandWhichImmediatelyRotatesToRotationOfLastPointInTrajectory(Trajectory trajectory, ProfiledPIDController robotAngleController) {
    return new SwerveControllerCommand(
      trajectory,
      this::getPose, // Functional interface to feed supplier
      m_kinematics,

      // Position controllers
      new PIDController(Constants.SWERVE_CONTROLLER_X_KP, 0, 0),
      new PIDController(Constants.SWERVE_CONTROLLER_Y_KP, 0, 0),
      robotAngleController,
      this::setModuleStates,
      this);
  }

  @Override
  public Command getMarkPositionCommand() {
    return new InstantCommand(() -> {
      var pos = this.getPose();
      _markedPosition = new Pose2d(pos.getTranslation(), pos.getRotation());
    });
  }

  @Override
  public Command getReturnToMarkedPositionCommand() {
    return new InstantCommand(() -> {
      var trajectoryConfig = this.GetTrajectoryConfig();
      var trajectory = TrajectoryGenerator.generateTrajectory(
        this.getPose(),
        List.of(),
        _markedPosition,
        trajectoryConfig);
  
      var cmd = this.CreateFollowTrajectoryCommandSwerveOptimized(trajectory);
      cmd.schedule();
    });
  }
  
  private void calculateDriveCharacteristics() {
    _driveCharacteristics.update(_odometryFromHardware.getPoseMeters(), 360 - m_navx.getAngle());
  }

  public double getFieldOrientedRobotMovementDirection(double x, double y) {
    // Use trigonometry to determine the angle at which the robot will be told to move by the controller.
    // Math.atan returns a result in radians.
    double radians = Math.atan(y / x);
    double degrees = Math.toDegrees(radians);

    return degrees;
  }

  // Return the difference between the robot's drive direction and its orientation,
  // in degrees. Needs checking to ensure that the offset is in the correct direction.
  public double getRobotDirectionRelativeToOrientation(double directionDegrees) {
      Rotation2d orientationObject = getGyroscopeRotation();
      double orientationDegrees = orientationObject.getDegrees();

      return directionDegrees - orientationDegrees;
  }
}