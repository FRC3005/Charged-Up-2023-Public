package frc.robot.constants;

public final class Intake {
  public static final int kIntakeCANId = 21;
  public static final boolean kMotorInvert = false;

  public static final double kConeInVoltage = 12.0;
  public static final double kConeOutVoltage = -1.5;
  public static final double kCubeInVoltage = -9.0;
  public static final double kCubeOutVoltage = 6.0;
  public static final double kConeHybridVoltage = -12.0;
  public static final double kCubeHybridVoltage = 6.0; // 9.0
  public static final double kCubeInVoltageAuton = -9.0;

  public static final double kGearReduction = 1.0;
  public static final double kVelocityFactor = 1.0 / kGearReduction;
}
