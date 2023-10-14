package frc.robot.constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import frc.lib.controller.PIDGains;
import frc.lib.util.RobotName;

public final class Arm {
  public static final int kLeaderId = 10;
  public static final int kFollowerId = 11;

  public static final PIDGains kGains = new PIDGains(0.75, 0.0, 0.13);
  public static final double kMaxVelocityRadPerSecond = 9.0;
  public static final double kMaxPositiveAccelerationRadPerSecSquared = 6.0;
  public static final double kMaxNegativeAccelerationRadPerSecSquared =
      RobotName.select(14.0, 14.0);

  public static final int kEncoderPort = 4;

  public static final ArmFeedforward kFeedForward = new ArmFeedforward(0.23, 0.1, 1.15, 0.081);
  // The offset of the arm from the horizontal in its neutral position,
  // measured from the horizontal

  // multiply SM value by this number and get arm position in radians
  public static final double kGearReduction = ((68.0 / 10.0) * (110.0 / 16.0));

  public static final double kArmMassKg = 10.0;
  public static final double kArmLengthM = Units.inchesToMeters(30.0);
  public static final double kMinRotation = Units.degreesToRadians(-48.0);
  public static final double kMaxRotation = Units.degreesToRadians(238.0);
  public static final double kAbsoluteSensorOffset = RobotName.select(3.400601, 4.338835);
  public static final boolean kAbsoluteSensorInvert = false;
  public static final boolean kMotorInvert = true;

  public static final double kArmFreeSpeed = 594.0 / kGearReduction;

  public static final float kForwardSoftLimit = 4.13f;
  public static final float kReverseSoftLimit = -1.05f;

  public static final double kPositionTolerance = 0.04;

  public static final double kHomePosition = -1.05;
  public static final double kSafePosition = -0.55;
  public static final double kMidCubePosition = 3.0;
  public static final double kHighCubePosition = 2.7;
  public static final double kMidConePosition = 2.92;
  public static final double kHighConePosition = 2.65;
  public static final double kCubeIntakePosition = -0.65;
  public static final double kConeIntakePosition = -0.881;
  public static final double kVerticalConeIntakePosition = -0.725;
  public static final double kMidReleasePosition = 3.11; // 3.07
  public static final double kHighReleasePosition = 2.86; // 2.8
  public static final double kTelescopeClearPosition =
      1.75; // Position where the telescope can start extending when going to scoring position
  public static final double kFrontMidConePosition = 0.75;
  public static final double kFrontMidCubePosition = 0.75;
  public static final double kFrontHighCubePosition = 0.282;
  public static final double kRearConeIntakePosition = Math.PI + 0.80;
  public static final double kRearCubeIntakePosition = Math.PI + 0.65;
  public static final double kHybridConePosition = -0.7;
  public static final double kHybridCubePosition = -0.55;
  public static final double kLolScoreCubeInAutoFromRampPosition = 0.588;
  public static final double kLowPresetOverlap = 0.2;
}
