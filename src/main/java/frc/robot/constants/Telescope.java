package frc.robot.constants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.controller.PIDGains;

public final class Telescope {
  public static final int kMotorId = 19;

  public static final double kMaxVelocityRadPerSecond = 5;
  public static final double kMaxAccelerationRadPerSecSquared = 8;

  public static final boolean kMotorInvert = false;

  public static final PIDGains kGains = new PIDGains(16.0, 0.0, 0.8);
  public static final TrapezoidProfile.Constraints kMotionConstraint =
      new TrapezoidProfile.Constraints(1.5, 10.0);

  public static final double kGearReduction = 64.0 / 13.0;
  public static final double kMetersPerRotation = (1.0 / kGearReduction) * 0.1016;
  public static final double kVelocityFactor = kMetersPerRotation / 60.0;
  public static final double kFreeSpeed = 5676.0 * kVelocityFactor;
  public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(0.0, 0.0);

  public static final float kForwardSoftLimit = 0.515f;
  public static final float kReverseSoftLimit = 0;

  public static final double kPositionTolerance = 0.01;

  public static final double kHomePosition = 0.0;
  public static final double kMidCubePosition = 0.0;
  public static final double kHighCubePosition = 0.30;
  public static final double kMidConePosition = 0.0;
  public static final double kHighConePosition = 0.45;
  public static final double kCubeIntakePosition = 0.27;
  public static final double kConeIntakePosition = 0.3125;
  public static final double kVerticalConeIntakePosition = 0.29;
  public static final double kPostScoreClearPosition =
      0.25; // The point where the wrist is okay to rotate when returning from the scoring position
  public static final double kFrontMidConePosition = 0.0;
  public static final double kFrontMidCubePosition = 0.0;
  public static final double kFrontHighCubePosition = 0.51;
  public static final double kHybridConePosition = 0.05;
  public static final double kHybridCubePosition = 0.1;
  public static final double kLowPresetOverlap = 0.1;
}
