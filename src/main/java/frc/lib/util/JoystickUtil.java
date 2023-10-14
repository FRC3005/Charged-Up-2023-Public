package frc.lib.util;

import edu.wpi.first.math.MathUtil;

public class JoystickUtil {
  public static double squareAxis(double input) {
    return (input * input) * Math.signum(input);
  }

  public static double squareAxis(double input, double deadband) {
    var inputAfterDeadband = MathUtil.applyDeadband(input, deadband);
    return (inputAfterDeadband * inputAfterDeadband) * Math.signum(input);
  }

  /**
   * Square the joystick axis and apply a scalar. This can be used to provide a squared joystick
   * response, with a deadband, scaled between some value e.g. something like 0.6 to provide 'fine
   * control'.
   *
   * @param input Joystick axis input, expected range [-1, 1]
   * @param deadband Joystick deadband range
   * @param outputScalar Scalar to multiply the axis value by
   * @return Scaled output to set as setpoint
   */
  public static double squareAxis(double input, double deadband, double outputScalar) {
    var inputAfterDeadband = MathUtil.applyDeadband(input, deadband);
    return (inputAfterDeadband * inputAfterDeadband) * Math.signum(input) * outputScalar;
  }
}
