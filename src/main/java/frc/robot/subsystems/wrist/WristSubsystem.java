// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

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
import frc.robot.constants.Wrist;

/** This is a sample program to demonstrate the use of elevator simulation with existing code. */
public class WristSubsystem extends SubsystemBase implements TelemetryNode {
  private final SparkMax m_motor;

  private final ProfiledPositionController m_controller;
  private final SimpleMotorFeedforward m_feedforward;
  private final Encoder m_neoEncoder;

  SimMech m_simMech;

  private boolean m_testmodeEnabled = false;
  private double m_setpoint = 0.0;

  public WristSubsystem(SimMech simMech) {
    m_simMech = simMech;
    m_motor = new SparkMax(Wrist.kMotorId).withInitializer(WristSubsystem::sparkMaxInitializer);
    m_controller =
        new ProfiledPositionController(
            m_motor.positionController(Wrist.kGains), Wrist.kMotionConstraint);

    m_neoEncoder = m_motor.builtinEncoder();

    m_feedforward = new SimpleMotorFeedforward(Wrist.kFeedforward);
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
    if (m_setpoint > Wrist.kForwardSoftLimit) {
      m_setpoint = Wrist.kForwardSoftLimit;
    } else if (m_setpoint < Wrist.kReverseSoftLimit) {
      m_setpoint = Wrist.kReverseSoftLimit;
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

  public double getPosition() {
    return m_neoEncoder.getPosition();
  }

  public boolean atSetpoint() {
    return Math.abs(getPosition() - m_setpoint) < Wrist.kPositionTolerance;
  }

  public Command runToPosition(double _setpoint, double _exitPoint) {
    CommandBase command =
        new CommandBase() {
          private double m_delta = Math.abs(_setpoint - _exitPoint);
          private boolean m_firstRun = true;

          public void initialize() {
            setPosition(_setpoint);
          }

          public void execute() {
            m_firstRun = false;
          }

          public void end(boolean interrupted) {
            /*if (interrupted) {
              setPosition(getPosition());
            }*/
          }

          public boolean isFinished() {
            return (atSetpoint() && !m_firstRun)
                || (Math.abs(getPosition() - _setpoint) <= m_delta);
          }
        };
    command.addRequirements(this);
    return command;
  }

  public Command runToPosition(double _setpoint) {
    return runToPosition(_setpoint, _setpoint);
  }

  private static Boolean sparkMaxInitializer(CANSparkMax sparkMax, Boolean isInit) {
    var encoder = sparkMax.getEncoder();
    int errors = 0;
    errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo500(sparkMax));
    errors += SparkMaxUtils.check(encoder.setPosition(0.0));
    errors +=
        SparkMaxUtils.check(
            sparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) Wrist.kForwardSoftLimit));
    errors +=
        SparkMaxUtils.check(
            sparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) Wrist.kReverseSoftLimit));

    errors += SparkMaxUtils.check(sparkMax.setIdleMode(IdleMode.kCoast));
    errors += SparkMaxUtils.check(encoder.setPositionConversionFactor(Wrist.kWristPositionFactor));
    errors += SparkMaxUtils.check(CANSparkMaxExtensions.setInverted(sparkMax, Wrist.kMotorInvert));
    SparkMax.setFrameStrategy(sparkMax, FrameStrategy.kVelocityAndPosition, true);
    return errors == 0;
  }

  public void bind(TelemetryBuilder builder) {
    builder.bindChild("NEO Encoder", m_neoEncoder);
    builder.bindChild("SparkMax", m_motor);
    builder.bindChild("Controller", m_controller);
    builder.addDoubleProperty("ks", () -> m_feedforward.ks, (ks) -> m_feedforward.ks = ks);
    builder.addDoubleProperty("kv", () -> m_feedforward.kv, (kv) -> m_feedforward.kv = kv);
    builder.addDoubleProperty("ka", () -> m_feedforward.ka, (ka) -> m_feedforward.ka = ka);

    builder.addBooleanProperty(
        "Testmode Enable", () -> m_testmodeEnabled, (e) -> m_testmodeEnabled = e);

    builder.addDoubleProperty("Setpoint", () -> m_setpoint, (s) -> m_setpoint = s);
  }
}
