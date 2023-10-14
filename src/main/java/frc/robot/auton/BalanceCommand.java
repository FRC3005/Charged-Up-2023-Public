// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.telemetry.TelemetryNode;
import frc.lib.vendor.sensor.ADIS16470;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.tinylog.Logger;

public class BalanceCommand extends CommandBase implements TelemetryNode {
  private DriveSubsystem m_drive;
  private final ADIS16470 m_imu;
  private double m_maxSpeedMs;
  private double m_output = 0.0;
  private int m_balancedCount = 0;
  private final double kBalancedValue = 0.07;
  private int m_startedBalancedCount = 60;
  private int m_delayCounts = 0;
  private int m_reverseCounts = 0;
  private double m_latchedOutput = 0.0;

  private final LinearFilter m_angleDeriv = LinearFilter.backwardFiniteDifference(1, 5, 0.02);

  private PIDController m_pid;
  /** Creates a new BalanceCommand. Note that the supplier MUST be angle positive when nose down */
  public BalanceCommand(DriveSubsystem drive, ADIS16470 imu, double maxSpeedMetersePerSecond) {
    m_drive = drive;
    m_imu = imu;
    m_maxSpeedMs = maxSpeedMetersePerSecond;
    addRequirements(drive);

    m_pid = new PIDController(0.45, 0, 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.tag("BalanceCommand").info("Starting balance command");

    if (Math.abs(m_imu.getAccelerationY()) < kBalancedValue) {
      m_startedBalancedCount = 0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // TODO: Change this to an ENUM state
    if (m_startedBalancedCount == 0) {
      m_startedBalancedCount++;
      return;
    } else if (m_startedBalancedCount == 1 && m_balancedCount != 0) {
      return;
    } else {
      m_startedBalancedCount++;
    }

    // Negate getAccelerationY() for angle using accel, or + getYAngle();
    var angle = -m_imu.getAccelerationY();
    double output = m_pid.calculate(angle, 0.0);

    if (Math.abs(angle) < kBalancedValue) {
      output = 0;
    }

    double gyroRate = m_imu.getRawYGyro();
    if (Math.abs(gyroRate) > 170.0 // Fast rate of change of gyro
        && Math.signum(gyroRate)
            == Math.signum(output) // Actively driving towards the direction of change
        && Math.signum(gyroRate) != Math.signum(angle) // Ramp starts falling/rising
        && m_delayCounts == 0) { // latch the output
      m_delayCounts = 55;
      m_reverseCounts = 5;
      m_latchedOutput = -output;
    }

    if (m_delayCounts > 0) {
      m_delayCounts--;

      if (m_reverseCounts > 0) {
        output = m_latchedOutput;
        m_reverseCounts--;
      } else {
        output = 0.0;
      }
    } else {
      m_reverseCounts = 0;
    }

    m_output = MathUtil.clamp(output, -m_maxSpeedMs, m_maxSpeedMs);
    m_drive.drive(m_output, 0.0, 0.0, false);
    SmartDashboard.putNumber("BALANCE OUTPUT", m_output);
    SmartDashboard.putNumber("BALANCE DELAY", m_delayCounts);
    SmartDashboard.putNumber("BALANCE REVERSE", m_reverseCounts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setX();
    Logger.tag("BalanceCommand").info("Swerve X!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This is time left in current mode
    double matchTime = DriverStation.getMatchTime();
    if (matchTime < 0.0) {
      matchTime = 100.0;
    }
    if (Math.abs(m_imu.getAccelerationY()) < kBalancedValue) {
      m_balancedCount++;
    } else {
      m_balancedCount = 0;
    }

    SmartDashboard.putNumber("Balanced Count", m_balancedCount);
    SmartDashboard.putNumber("Match Time Bal", matchTime);
    // 20ms * this value

    // Match timer in practice mode is a floating point, but on the field
    // is an int value that doesn't get updated too frequently. So best we
    // can do is check for the 1s mark, which may actually come in late.

    // Looking at match data, it looks like there is 300ms available with
    // the match time = 0. It also sometimes rolls over to teleop time
    // in auton so check for that case too. Use 0.3 to 'simulate' this in
    // practice mode.
    return m_balancedCount >= 100 || matchTime <= 0.3 || matchTime >= 100;
  }
}
