package frc.lib.telemetry;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class DataTypes {

  public static double[] swerveStatesToData(SwerveModuleState states[]) {
    double[] result = new double[states.length * 2];
    int i = 0;
    for (var state : states) {
      result[i++] = state.angle.getRadians();
      result[i++] = state.speedMetersPerSecond;
    }
    return result;
  }
}
