// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxExtensions;
import com.revrobotics.CANSparkMaxSim1;
import com.revrobotics.SimDynamics;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.controller.ArmFeedforward;
import frc.lib.controller.AsymmetricTrapezoidProfile;
import frc.lib.controller.Controller;
import frc.lib.controller.ProfiledPositionController;
import frc.lib.electromechanical.Encoder;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;
import frc.lib.util.Faults;
import frc.lib.vendor.motorcontroller.SparkMax;
import frc.lib.vendor.motorcontroller.SparkMax.FrameStrategy;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import frc.lib.vendor.sensor.ThroughBoreEncoder;
import frc.robot.Robot;
import frc.robot.SimMech;
import frc.robot.constants.Arm;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmSubsystem extends SubsystemBase implements TelemetryNode {
  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor m_armGearbox = DCMotor.getNEO(2);

  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          m_armGearbox,
          Arm.kGearReduction,
          SingleJointedArmSim.estimateMOI(Arm.kArmLengthM, Arm.kArmMassKg),
          Arm.kArmLengthM,
          Arm.kMinRotation,
          Arm.kMaxRotation,
          // Arm.kArmMassKg,
          false, // Gas shock makes gravity significantly less
          VecBuilder.fill(
              2.0 * Math.PI / (42 * Arm.kGearReduction * 100)) // Add noise with a std-dev of 1 tick
          );

  private final SparkMax m_motor;
  private final SparkMax m_follower;
  private final CANSparkMaxSim1 m_motorSim;
  private boolean m_jogged = false;

  private ArmFeedforward m_feedforward = new ArmFeedforward(Arm.kFeedForward);
  public static final AsymmetricTrapezoidProfile.Constraints kPositiveConstraint =
      new AsymmetricTrapezoidProfile.Constraints(
          Arm.kMaxVelocityRadPerSecond,
          Arm.kMaxNegativeAccelerationRadPerSecSquared,
          Arm.kMaxPositiveAccelerationRadPerSecSquared);
  public static final AsymmetricTrapezoidProfile.Constraints kSlowConstraint =
      new AsymmetricTrapezoidProfile.Constraints(
          Arm.kMaxVelocityRadPerSecond,
          Arm.kMaxPositiveAccelerationRadPerSecSquared,
          Arm.kMaxPositiveAccelerationRadPerSecSquared);
  public static final AsymmetricTrapezoidProfile.Constraints kFastConstraints =
      new AsymmetricTrapezoidProfile.Constraints(
          Arm.kMaxVelocityRadPerSecond,
          Arm.kMaxNegativeAccelerationRadPerSecSquared,
          Arm.kMaxNegativeAccelerationRadPerSecSquared);

  private final ProfiledPositionController m_controller;

  private SimMech m_simMech = new SimMech();

  private final ThroughBoreEncoder m_absoluteEncoder =
      new ThroughBoreEncoder(
          Arm.kEncoderPort, Arm.kAbsoluteSensorOffset, 2 * Math.PI, Arm.kAbsoluteSensorInvert);
  private final Encoder m_neoEncoder;

  private boolean m_testmodeEnabled = false;

  private double m_setpoint;

  /** Create a new ArmSubsystem. */
  public ArmSubsystem(SimMech simMech) {
    m_follower = new SparkMax(Arm.kFollowerId);
    m_motor =
        new SparkMax(Arm.kLeaderId)
            .withInitializer(ArmSubsystem::sparkMaxInitializer)
            .withFollower(m_follower);

    Controller sparkMaxPositionController = m_motor.positionController(Arm.kGains);
    m_controller = new ProfiledPositionController(sparkMaxPositionController, kPositiveConstraint);
    m_motorSim = new CANSparkMaxSim1(m_motor, SimDynamics.fromSim(m_armSim));
    m_neoEncoder = m_motor.builtinEncoder();

    if (!m_absoluteEncoder.waitForInit(10.0)) {
      Faults.subsystem("Arm").fatal("Arm absolute encoder failed! Check encoder and reset");
      // TODO: Set the 'stowed' position instead of the abs sensor
      resetEncoder();
    } else {
      resetEncoder();
    }
  }

  public boolean atGoal() {
    return atGoal(100000);
  }

  public boolean atGoal(double tolerance) {
    if (Math.abs(m_absoluteEncoder.getPosition() - m_setpoint) < tolerance) {
      return m_controller.atGoal();
    }
    return false;
  }

  public void jog(double amount) {
    m_setpoint += amount;
    m_jogged = true;
  }

  @Override
  public void periodic() {
    if (m_setpoint > Arm.kForwardSoftLimit) {
      m_setpoint = Arm.kForwardSoftLimit;
    } else if (m_setpoint < Arm.kReverseSoftLimit) {
      m_setpoint = Arm.kReverseSoftLimit;
    }

    if (m_jogged) {
      m_controller.setConstraints(kSlowConstraint);
    } else if (m_controller.getInitialTarget().position < 0.0
        && m_controller.getGoal().position < 0.0) {
      m_controller.setConstraints(kFastConstraints);
    } else if (m_controller.getProfileDirection() < 0.0) {
      m_controller.setConstraints(kSlowConstraint);
    } else if (m_controller.getProfileDirection() > 0.0) {
      m_controller.setConstraints(kPositiveConstraint);
    } /* Keep the constraint the same for direciton = 0 */

    m_controller.setReference(
        m_setpoint,
        getPositionRadians(),
        (setpoint) -> m_feedforward.calculate(setpoint.position, setpoint.velocity));
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

  private double getAbsoluteSensorPosition() {
    double val = m_absoluteEncoder.getPosition();

    // Wrap after far extreme, so the range go from [0, 360] to
    // [-90, 270] to match the arm range.
    if (val > Units.degreesToRadians(270.0)) {
      val = val - Units.degreesToRadians(360.0);
    }

    return val;
  }

  public double getPositionRadians() {
    return m_neoEncoder.getPosition();
  }

  public void setPosition(double goal) {
    m_setpoint = goal;
    m_jogged = false;
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

          public void execute() {
            // m_firstRun = false;
          }

          public void end(boolean interrupted) {
            /*if (interrupted) {
              setPosition(getPositionRadians());
            }*/
          }

          public boolean isFinished() {
            boolean finished =
                (atSetpoint() && !m_firstRun)
                    || (Math.abs(getPositionRadians() - _setpoint) <= m_delta);
            m_firstRun = false;
            return finished;
          }
        };
    command.addRequirements(this);
    return command;
  }

  public Command runToPosition(double _setpoint) {
    return runToPosition(_setpoint, _setpoint);
  }

  public Command telescopeClear() {
    CommandBase command =
        new CommandBase() {
          public boolean isFinished() {
            return getPositionRadians() > Arm.kTelescopeClearPosition;
          }
        };
    // command.addRequirements(this);
    return command;
  }

  public Command armClearLess(double angle) {
    CommandBase command =
        new CommandBase() {
          public boolean isFinished() {
            return getPositionRadians() < angle;
          }
        };
    return command;
  }

  public Command armClearGreater(double angle) {
    CommandBase command =
        new CommandBase() {
          public boolean isFinished() {
            return getPositionRadians() > angle;
          }
        };
    return command;
  }

  public Command scheduleWhenArmLess(Command command, double value) {
    return new CommandBase() {
      public boolean isFinished() {
        return getPositionRadians() < value;
      }
    }.andThen(command);
  }

  public Command scheduleWhenArmGreater(Command command, double value) {
    return new CommandBase() {
      public boolean isFinished() {
        return getPositionRadians() > value;
      }
    }.andThen(command);
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    double voltage = RobotController.getBatteryVoltage();
    m_armSim.setInput(m_motor.getAppliedOutput() * voltage);

    // Next, we update it. The standard loop time is 20ms.
    m_motorSim.iterate(voltage, 0.020);
    m_armSim.update(0.020);
    m_motorSim.setPosition(m_armSim.getAngleRads());

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_simMech.setArm(m_armSim.getAngleRads());
  }

  private static Boolean sparkMaxInitializer(CANSparkMax sparkMax, Boolean isInit) {
    var encoder = sparkMax.getEncoder();
    int errors = 0;
    errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo(sparkMax));
    errors +=
        SparkMaxUtils.check(
            sparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) Arm.kForwardSoftLimit));
    errors +=
        SparkMaxUtils.check(
            sparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) Arm.kReverseSoftLimit));

    errors +=
        SparkMaxUtils.check(encoder.setPositionConversionFactor(2 * Math.PI / Arm.kGearReduction));
    errors += SparkMaxUtils.check(CANSparkMaxExtensions.setInverted(sparkMax, Arm.kMotorInvert));
    SparkMax.setFrameStrategy(sparkMax, FrameStrategy.kVelocityAndPosition, true);
    return errors == 0;
  }

  public void resetEncoder() {
    m_setpoint = getAbsoluteSensorPosition();

    /*
     * Disable the arm motor briefly such that the arm doesn't
     * jump while waiting for the setpoint to be set.
     */
    m_motor.set(0);
    m_neoEncoder.setPosition(m_setpoint);
    m_neoEncoder.setPosition(m_setpoint);
    Timer.delay(0.005);
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    builder.bindChild("NEO Encoder", m_neoEncoder);
    builder.bindChild("Through Bore", m_absoluteEncoder);
    builder.bindChild("Motor", m_motor);
    builder.bindChild("Follower", m_follower);
    builder.addDoubleProperty("Through Bore Angle", this::getAbsoluteSensorPosition, null);
    builder.bindChild("Controller", m_controller);
    builder.addBooleanProperty(
        "Test mode enable", () -> m_testmodeEnabled, (enable) -> m_testmodeEnabled = enable);
    builder.addDoubleProperty("Setpoint", () -> m_setpoint, this::setPosition);

    builder.addBooleanProperty("Reset Encoder", () -> false, (val) -> resetEncoder());

    builder.addDoubleProperty("ks", () -> m_feedforward.ks, (ks) -> m_feedforward.ks = ks);
    builder.addDoubleProperty("kg", () -> m_feedforward.kg, (kg) -> m_feedforward.kg = kg);
    builder.addDoubleProperty("kv", () -> m_feedforward.kv, (kv) -> m_feedforward.kv = kv);
    builder.addDoubleProperty("ka", () -> m_feedforward.ka, (ka) -> m_feedforward.ka = ka);

    if (Robot.isSimulation()) {
      builder.addDoubleProperty("Sim Arm Angle", () -> m_armSim.getAngleRads(), null);
    }
  }
}
