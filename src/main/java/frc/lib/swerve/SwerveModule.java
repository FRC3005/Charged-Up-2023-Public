// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.telemetry.TelemetryNode;

public interface SwerveModule extends TelemetryNode {
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState();

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState);

  /**
   * Set the module to keep its current heading, with a certain speed. Useful for holding rotation
   * e.g. when the drive sticks are not pressed.
   *
   * @param speedMetersPerSecond the speed to set the module to.
   */
  public void holdHeading(double speedMetersPerSecond);

  /** Reset the drive encoder to zero. */
  public void resetEncoders();

  /**
   * Get the module position. This is the same as getState, except with the position of the wheel
   * instead of velocity.
   *
   * @return Current module position.
   */
  public SwerveModulePosition getPosition();

  /**
   * Periodic funcion runs at the rate of the swerve drive. Used in the case that a PID controller
   * must be run continuously or similar.
   */
  public default void periodic() {}

  public default void simulationPeriodic() {}

  public default void testPeriodic() {}

  public double getDriveDistanceMeters();

  public SwerveModuleState getActualState();
}
