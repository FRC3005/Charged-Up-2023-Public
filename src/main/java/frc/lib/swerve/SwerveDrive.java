// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.swerve;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.odometry.CarpetTransform;
import frc.lib.odometry.SwerveDriveCarpetOdometry;
import frc.lib.telemetry.DataTypes;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;
import frc.lib.vendor.sensor.ADIS16470;
import frc.robot.constants.Drivetrain;
import frc.robot.constants.Field;
import frc.robot.constants.RobotConstants;
import java.util.function.Supplier;
import org.tinylog.Logger;

public abstract class SwerveDrive extends SubsystemBase implements TelemetryNode {
  public enum ModuleLocation {
    frontLeft(0),
    frontRight(1),
    rearLeft(2),
    rearRight(3);

    public final int value;
    private static final ModuleLocation[] m_mapping =
        new ModuleLocation[] {frontLeft, frontRight, rearLeft, rearRight};

    private ModuleLocation(int v) {
      this.value = v;
    }

    public static ModuleLocation fromInt(int v) {
      return m_mapping[v];
    }
  }
  // Robot swerve modules
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_rearLeft;
  private final SwerveModule m_rearRight;

  // The gyro sensor
  private final ADIS16470 m_gyro;

  private final SwerveDriveKinematics m_kinematics;
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;
  SwerveDriveCarpetOdometry m_carpetOdometry;

  private final CarpetTransform m_transform =
      new CarpetTransform(Drivetrain.kCarpetLookup, Field.HomeField.kCompetitionCarpet);

  private final Field2d m_field = new Field2d();
  private final Field2d m_field2 = new Field2d();

  private final double m_maxSpeed;

  /**
   * Create a Swerve Drive module
   *
   * @param frontLeft Swerve Module
   * @param frontRight Swerve Module
   * @param rearLeft Swerve Module
   * @param rearRight Swerve Module
   * @param kinematics Swerve drive kinematics
   * @param gyro used for odometry and field centric driving
   * @param maxSpeed of the wheels used to normalize wheel speeds
   */
  public SwerveDrive(
      SwerveModule frontLeft,
      SwerveModule frontRight,
      SwerveModule rearLeft,
      SwerveModule rearRight,
      SwerveDriveKinematics kinematics,
      ADIS16470 gyro,
      double maxSpeed) {
    m_frontLeft = frontLeft;
    m_frontRight = frontRight;
    m_rearLeft = rearLeft;
    m_rearRight = rearRight;
    m_gyro = gyro;
    m_kinematics = kinematics;
    m_maxSpeed = maxSpeed;
    m_odometry = new SwerveDriveOdometry(kinematics, m_gyro.getRotation2d(), getModulePositions());
    m_carpetOdometry =
        new SwerveDriveCarpetOdometry(
            Drivetrain.kDriveCarpetKinematics,
            m_gyro.getRotation2d(),
            getModulePositions(),
            m_transform);
    configurePathPlannerTelemetry();
  }

  private ChassisSpeeds m_chassisSpeed = new ChassisSpeeds();

  private double m_cor_x = 0.0;
  private double m_cor_y = 0.0;

  @Override
  public void bind(TelemetryBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Velocity vx", () -> m_chassisSpeed.vxMetersPerSecond, null);
    builder.addDoubleProperty("Velocity vy", () -> m_chassisSpeed.vyMetersPerSecond, null);
    builder.addDoubleProperty("Velocity omega", () -> m_chassisSpeed.omegaRadiansPerSecond, null);
    builder.bindChild("Gyro", m_gyro);
    builder.bindChild("frontLeft", m_frontLeft);
    builder.bindChild("frontRight", m_frontRight);
    builder.bindChild("rearLeft", m_rearLeft);
    builder.bindChild("rearRight", m_rearRight);
    builder.bindSendableChild("Field 2d", m_field);
    builder.addDoubleArrayProperty(
        "Swerve State", () -> DataTypes.swerveStatesToData(getModuleStates()), null);

    if (!RobotConstants.kReducedTelemetryMode) {
      builder.bindSendableChild("Field 2d Carpet", m_field2);
      builder.addDoubleArrayProperty(
          "Swerve SM State", () -> DataTypes.swerveStatesToData(getActualModuleStates()), null);
    }
  }

  @Override
  public void periodic() {
    // Run swerve modules if they require it
    m_frontLeft.periodic();
    m_frontRight.periodic();
    m_rearLeft.periodic();
    m_rearRight.periodic();

    m_chassisSpeed = m_kinematics.toChassisSpeeds(getModuleStates());

    SwerveModulePosition[] module_positions =
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition(),
        };

    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), module_positions);
    m_carpetOdometry.update(m_gyro.getRotation2d(), module_positions);
    m_field.setRobotPose(m_odometry.getPoseMeters());
    m_field2.setRobotPose(m_carpetOdometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    m_frontLeft.simulationPeriodic();
    m_frontRight.simulationPeriodic();
    m_rearRight.simulationPeriodic();
    m_rearLeft.simulationPeriodic();
  }

  /**
   * Return the current velocity of the chassis as a ChassisSpeeds object.
   *
   * @return velocity of the robot.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_chassisSpeed;
  }

  /**
   * Predict the motion between the current position and a future state.
   *
   * @param lookahead Time in seconds to predict ahead.
   * @return twist2d represnting the change in pose over the lookahead time.
   */
  public Twist2d getPredictedMotion(double lookahead) {
    ChassisSpeeds chassisSpeed = m_kinematics.toChassisSpeeds(getModuleStates());
    return new Twist2d(
        chassisSpeed.vxMetersPerSecond * lookahead,
        chassisSpeed.vyMetersPerSecond * lookahead,
        chassisSpeed.omegaRadiansPerSecond * lookahead);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
    m_carpetOdometry.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // If nothing is commanded, hold the same position
    if (xSpeed == 0.0 && ySpeed == 0.0 && rot == 0.0) {
      holdAllModulesRotation(0.0);
      return;
    }

    ySpeed = ySpeed * m_maxSpeed;
    xSpeed = xSpeed * m_maxSpeed;
    rot = rot * m_maxSpeed;

    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            new Translation2d(m_cor_x, m_cor_y));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_maxSpeed);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  private void holdAllModulesRotation(double speedMetersPerSecond) {
    m_frontLeft.holdHeading(speedMetersPerSecond);
    m_frontRight.holdHeading(speedMetersPerSecond);
    m_rearLeft.holdHeading(speedMetersPerSecond);
    m_rearRight.holdHeading(speedMetersPerSecond);
  }

  /** Set the swerve drive into an X which is not drivable but should help prevent being pushed. */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, m_maxSpeed);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Set an individual module state independently of all other modules. This should only be used for
   * testing/tuning.
   */
  public void testPeriodic() {
    m_frontLeft.testPeriodic();
    m_frontRight.testPeriodic();
    m_rearLeft.testPeriodic();
    m_rearRight.testPeriodic();
  }

  public SwerveModuleState[] getModuleStates() {
    // the order of this array MUST match the array in the drive constants
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState(),
    };
  }

  public SwerveModuleState[] getActualModuleStates() {
    // the order of this array MUST match the array in the drive constants
    return new SwerveModuleState[] {
      m_frontLeft.getActualState(),
      m_frontRight.getActualState(),
      m_rearLeft.getActualState(),
      m_rearRight.getActualState(),
    };
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    resetOdometry(new Pose2d(m_odometry.getPoseMeters().getTranslation(), new Rotation2d()));
  }

  /** Calibrate the gyro. Requirements for the device being still depend on the gyro being used. */
  public void calibrateGyro() {
    m_gyro.calibrate();
  }

  public void setHeading(double degreesCCWPositive) {
    Logger.tag("Swerve Drive").trace("Setting heading to {} degrees", degreesCCWPositive);
    m_gyro.setAngle(degreesCCWPositive);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate();
  }

  public void stop() {
    drive(0.0, 0.0, 0.0, false);
  }

  public Command stopCommand() {
    return new RunCommand(this::stop, this).withName("Swerve Stop");
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition(),
    };
  }
  /**
   * Create a trajectory following command. Note that the beginning and end states of the command
   * are not necessarily 0 speed.
   *
   * @param trajectory PathPlanner trajectory
   * @param xController PID Controller for the X direction (left/right)
   * @param yController PID Contorller for the Y direction (forward/back)
   * @param thetaController Turning PID Controller for rotation (CCW Positive)
   * @return Command to be scheduled
   */
  public Command trajectoryFollowerCommand(
      PathPlannerTrajectory trajectory,
      PIDController xController,
      PIDController yController,
      PIDController thetaController) {
    return trajectoryFollowerCommand(
        trajectory, this::getPose, xController, yController, thetaController);
  }

  /**
   * Create a trajectory following command. Note that the beginning and end states of the command
   * are not necessarily 0 speed.
   *
   * @param trajectory PathPlanner trajectory
   * @param xController PID Controller for the X direction (left/right)
   * @param yController PID Contorller for the Y direction (forward/back)
   * @param thetaController Turning PID Controller for rotation (CCW Positive)
   * @return Command to be scheduled
   */
  public Command trajectoryFollowerCommand(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      PIDController xController,
      PIDController yController,
      PIDController thetaController) {
    Command swCommand =
        new PPSwerveControllerCommand(
            trajectory,
            poseSupplier,
            m_kinematics,
            xController,
            yController,
            thetaController,
            (states) -> setModuleStates(states),
            true,
            this);
    return new InstantCommand(() -> m_field.getObject("Trajectory").setTrajectory(trajectory))
        .alongWith(swCommand);
  }

  /**
   * Add functions to log telemetry for any pathing to NT. Keep these separate from the 'bind'
   * function so they only run when a command is actually active.
   */
  private void configurePathPlannerTelemetry() {
    PPSwerveControllerCommand.setLoggingCallbacks(
        this::PPLogTrajectory, this::PPLogTargetPose, this::PPLogSetpoint, this::PPLogError);
  }

  private FieldObject2d m_trajectory = m_field.getObject("Active Trajectory");

  private void PPLogTrajectory(PathPlannerTrajectory trajectory) {
    m_trajectory.setTrajectory(trajectory);
  }

  private FieldObject2d m_targetPose = m_field.getObject("Target Pose");

  private void PPLogTargetPose(Pose2d pose) {
    m_targetPose.setPose(pose);
  }

  private void PPLogSetpoint(ChassisSpeeds setpoint) {
    SmartDashboard.putNumber("PPSwerveControllerCommand/vxSetpoint", setpoint.vxMetersPerSecond);
    SmartDashboard.putNumber("PPSwerveControllerCommand/vySetpoint", setpoint.vyMetersPerSecond);
    SmartDashboard.putNumber(
        "PPSwerveControllerCommand/rotSetpoint", setpoint.omegaRadiansPerSecond);
  }

  private void PPLogError(Translation2d translationError, Rotation2d rotationError) {
    SmartDashboard.putNumber("PPSwerveControllerCommand/xErrorMeters", translationError.getX());
    SmartDashboard.putNumber("PPSwerveControllerCommand/yErrorMeters", translationError.getY());
    SmartDashboard.putNumber(
        "PPSwerveControllerCommand/rotationErrorRadians", rotationError.getRadians());
  }
}
