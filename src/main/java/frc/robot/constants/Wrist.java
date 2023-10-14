package frc.robot.constants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.controller.PIDGains;
import frc.lib.vendor.constants.REV;

public final class Wrist {
  public static final int kMotorId = 18;
  public static final PIDGains kGains = new PIDGains(10.0, 0.0, 1.0);
  public static final TrapezoidProfile.Constraints kMotionConstraint =
      new TrapezoidProfile.Constraints(18.0, 200.0);

  public static final boolean kMotorInvert = false;

  public static float kForwardSoftLimit = (float) (2.0 * Math.PI);
  public static float kReverseSoftLimit = (float) (-0.5 * Math.PI);

  public static final double kWristGearRatio =
      REV.UltraPlanetaryConstants.kThreeToOneRatio
          * REV.UltraPlanetaryConstants.kFourToOneRatio
          * (11.0 / 50.0);

  public static final double kTmp =
      (REV.UltraPlanetaryConstants.kThreeToOneRatio
          * REV.UltraPlanetaryConstants.kFourToOneRatio
          * (11.0 / 50.0));
  public static final double kWristFreeSpeed = REV.Neo550MotorConstants.kFreeSpeedRadPerSec * kTmp;
  public static final double kWristPositionFactor = kWristGearRatio * 2 * Math.PI;
  public static final double kWristVelocityFactor = kWristPositionFactor / 60.0;
  public static final SimpleMotorFeedforward kFeedforward =
      new SimpleMotorFeedforward(0.0, 12.0 / kWristFreeSpeed, 0.0);

  public static final double kPositionTolerance = 0.2; // 0.02

  public static final double kCubeIntakePosition = 0.0;
  public static final double kConeIntakePosition = Math.PI;
  public static final double kLeftPosition = 1.5 * Math.PI;
  public static final double kRightPosition = 0.5 * Math.PI;
}
