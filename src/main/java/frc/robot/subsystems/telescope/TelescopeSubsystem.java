// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.telescope;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxExtensions;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.controller.ProfiledPositionController;
import frc.lib.controller.SimpleMotorFeedforward;
import frc.lib.electromechanical.Encoder;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;
import frc.lib.vendor.motorcontroller.SparkMax;
import frc.lib.vendor.motorcontroller.SparkMax.FrameStrategy;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import frc.robot.Robot;
import frc.robot.SimMech;
import frc.robot.constants.Telescope;

/** This is a sample program to demonstrate the use of elevator simulation with existing code. */
public class TelescopeSubsystem extends SubsystemBase implements TelemetryNode {
  private final SparkMax m_motor;

  private final Encoder m_neoEncoder;

  private final ProfiledPositionController m_controller;

  private final SimpleMotorFeedforward m_feedforward;

  private boolean m_testmodeEnabled = false;

  SimMech m_simMech;

  private double m_setpoint = 0.0;

  public TelescopeSubsystem(SimMech simMech) {
    m_simMech = simMech;
    m_motor =
        new SparkMax(Telescope.kMotorId).withInitializer(TelescopeSubsystem::sparkMaxInitializer);
    var neoController = m_motor.positionController(Telescope.kGains);
    m_controller = new ProfiledPositionController(neoController, Telescope.kMotionConstraint);

    m_neoEncoder = m_motor.builtinEncoder();

    m_feedforward = new SimpleMotorFeedforward(Telescope.kFeedforward);
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    // m_elevatorSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    // m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    // m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    // RoboRioSim.setVInVoltage(
    // BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

    // Update elevator visualization with simulated position
    // m_simMech.m_elevator.setLength(Units.metersToInches(m_elevatorSim.getPositionMeters()));
    // SmartDashboard.putNumber("Elevator Length", m_elevatorSim.getPositionMeters());
  }

  @Override
  public void periodic() {
    if (m_setpoint > Telescope.kForwardSoftLimit) {
      m_setpoint = Telescope.kForwardSoftLimit;
    } else if (m_setpoint < Telescope.kReverseSoftLimit) {
      m_setpoint = Telescope.kReverseSoftLimit;
    }

    m_controller.setReference(
        m_setpoint, getPosition(), (setpoint) -> m_feedforward.calculate(setpoint.velocity));
  }

  public void testModePeriodic() {
    if (m_testmodeEnabled) {
      periodic();
    } else {
      m_motor.set(0);
    }

    if (Robot.isSimulation()) {
      simulationPeriodic();
    }
  }

  public void setPosition(double setpoint) {
    m_setpoint = setpoint;
  }

  public void jog(double amount) {
    m_setpoint += amount;
  }

  public double getPosition() {
    return m_neoEncoder.getPosition();
  }

  public boolean atSetpoint() {
    return m_controller.atGoal();
  }

  public Command runToPosition(double _setpoint, double _exitPoint) {
    CommandBase command =
        new CommandBase() {
          private double m_delta = Math.abs(_setpoint - _exitPoint);
          private boolean m_firstRun = true;

          public void initialize() {
            setPosition(_setpoint);
          }

          public void execute() {}

          public void end(boolean interrupted) {
            /*if (interrupted) {
              setPosition(getPosition());
            }*/
          }

          public boolean isFinished() {
            boolean isFinished =
                (atSetpoint() && !m_firstRun) || (Math.abs(getPosition() - _setpoint) <= m_delta);
            m_firstRun = false;
            return isFinished;
          }
        };
    command.addRequirements(this);
    return command;
  }

  public Command runToPosition(double _setpoint) {
    return runToPosition(_setpoint, _setpoint);
  }

  public Command wristClear() {
    CommandBase command =
        new CommandBase() {
          public boolean isFinished() {
            return getPosition() < Telescope.kPostScoreClearPosition;
          }
        };
    return command;
  }

  private static Boolean sparkMaxInitializer(CANSparkMax sparkMax, Boolean isInit) {
    var encoder = sparkMax.getEncoder();
    int errors = 0;
    errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo(sparkMax));
    errors += SparkMaxUtils.check(encoder.setPosition(0.0));
    errors +=
        SparkMaxUtils.check(
            sparkMax.setSoftLimit(
                SoftLimitDirection.kForward, (float) Telescope.kForwardSoftLimit));
    errors +=
        SparkMaxUtils.check(
            sparkMax.setSoftLimit(
                SoftLimitDirection.kReverse, (float) Telescope.kReverseSoftLimit));

    errors +=
        SparkMaxUtils.check(encoder.setPositionConversionFactor(Telescope.kMetersPerRotation));
    errors +=
        SparkMaxUtils.check(CANSparkMaxExtensions.setInverted(sparkMax, Telescope.kMotorInvert));
    errors += SparkMaxUtils.check(sparkMax.setIdleMode(IdleMode.kBrake));
    SparkMax.setFrameStrategy(sparkMax, FrameStrategy.kVelocityAndPosition);
    return errors == 0;
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    builder.bindChild("Neo Encoder", m_neoEncoder);
    builder.bindChild("Motor", m_motor);
    builder.bindChild("Controller", m_controller);
    builder.addDoubleProperty("Setpoint", () -> m_setpoint, (s) -> m_setpoint = s);
    builder.addDoubleProperty("ks", () -> m_feedforward.ks, (ks) -> m_feedforward.ks = ks);
    builder.addDoubleProperty("kv", () -> m_feedforward.kv, (kv) -> m_feedforward.kv = kv);
    builder.addDoubleProperty("ka", () -> m_feedforward.ka, (ka) -> m_feedforward.ka = ka);
    builder.addBooleanProperty(
        "Test Mode Enable", () -> m_testmodeEnabled, (e) -> m_testmodeEnabled = e);
  }
}
