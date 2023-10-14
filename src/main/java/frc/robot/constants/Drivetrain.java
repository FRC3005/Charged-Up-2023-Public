package frc.robot.constants;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.controller.PIDGains;
import frc.lib.odometry.SwerveDriveCarpetKinematics;
import java.util.Arrays;
import java.util.List;

public final class Drivetrain {
  public static final int kFrontLeftDriveCanId = 3;
  public static final int kRearLeftDriveCanId = 7;
  public static final int kFrontRightDriveCanId = 1;
  public static final int kRearRightDriveCanId = 5;

  public static final int kFrontLeftTurningCanId = 4;
  public static final int kRearLeftTurningCanId = 8;
  public static final int kFrontRightTurningCanId = 2;
  public static final int kRearRightTurningCanId = 6;

  // Drive motor current limit in Amps
  public static final int kDriveMotorCurrentLimit = 50;

  // Max Velocity in m/s
  public static final double kTurningMotorMaxVelocity = 5.0;

  // Max acceleration in m/s^2
  public static final double kTurningMotorMaxAccel = 5.0;
  public static final double kMaxDriveSpeed = 5.0;

  public static PIDGains kDriveMotorPIDGains = new PIDGains(0.18, 0.0, 0.001);
  public static final PIDGains kTurningMotorPIDGains = new PIDGains(2.138, 0.00, 0.2);

  public static final double kTrackWidth = Units.inchesToMeters(19.5);
  public static final double kWheelBase = Units.inchesToMeters(22.5);
  public static final SwerveDriveKinematics kDriveKinematics =
      new SwerveDriveKinematics(
          new Translation2d(kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
  public static final SwerveDriveCarpetKinematics kDriveCarpetKinematics =
      new SwerveDriveCarpetKinematics(
          new Translation2d(kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

  // Number of rotations of the motor / number of rotations of the output
  // 45 : 22 then 14 : 15
  public static final double kDriveMotorReduction = (45.0 * 22.0) / (14.0 * 15.0);

  public static final double kWheelDiameterMeters = Units.inchesToMeters(3.0);

  // Assumes the encoders are directly mounted on the wheel shafts
  public static final double kDriveEncoderPositionFactor =
      (kWheelDiameterMeters * Math.PI) / kDriveMotorReduction;

  // public static final double kDriveEncoderPositionFactor = 1.0 / kDriveMotorReduction;

  public static final double kDriveEncoderVelocityFactor =
      // Assumes the encoders are directly mounted on the wheel shafts
      ((kWheelDiameterMeters * Math.PI) / (double) kDriveMotorReduction) / 60.0;
  public static final boolean kDriveMotorInvert = false;

  public static final double kTurningModuleGearRatio = 9424.0 / 203.0;
  public static final double kTurningEncoderPositionFactor =
      (2 * Math.PI) / kTurningModuleGearRatio;
  public static final double kTurningEncoderVelocityFactor =
      ((2 * Math.PI) / kTurningModuleGearRatio) / 60.0;
  public static final boolean kTurningMotorInvert = false;
  public static final SimpleMotorFeedforward kDriveFeedforward =
      new SimpleMotorFeedforward(0.3, 2.5, 0.9);
  public static final SimpleMotorFeedforward kTurningFeedforward =
      new SimpleMotorFeedforward(0.35233, 0.39185, 0.0058658);
  public static final TrapezoidProfile.Constraints kTurningConstraints =
      new TrapezoidProfile.Constraints(20, 200);

  public static final int kFrontLeftAbsoluteEncoderPort = 0;
  public static final int kFrontRightAbsoluteEncoderPort = 3;
  public static final int kRearLeftAbsoluteEncoderPort = 1;
  public static final int kRearRightAbsoluteEncoderPort = 2;

  // Auton path finding controllers
  public static final PIDController kXController = new PIDController(1.0, 0.0, 0.0);
  public static final PIDController kYController = new PIDController(1.0, 0.0, 0.0);

  // High profile constraints = pure P controller
  public static PIDController kThetaController = new PIDController(5.0, 0.0, 0.8);

  public static final List<Pair<Rotation2d, Double>> kCarpetLookup =
      Arrays.asList(
          Pair.of(Rotation2d.fromDegrees(-180), 0.955),
          Pair.of(Rotation2d.fromDegrees(-135), 0.967),
          Pair.of(Rotation2d.fromDegrees(-90), 1.009),
          Pair.of(Rotation2d.fromDegrees(-45), 1.026),
          Pair.of(Rotation2d.fromDegrees(0), 1.034),
          Pair.of(Rotation2d.fromDegrees(45), 1.015),
          Pair.of(Rotation2d.fromDegrees(90), 1.012),
          Pair.of(Rotation2d.fromDegrees(135), 0.973),
          Pair.of(Rotation2d.fromDegrees(180), 0.955));
}
