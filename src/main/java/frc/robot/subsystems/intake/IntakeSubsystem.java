package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxExtensions;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.electromechanical.Encoder;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;
import frc.lib.vendor.constants.REV.Neo550MotorConstants;
import frc.lib.vendor.motorcontroller.SparkMax;
import frc.lib.vendor.motorcontroller.SparkMax.FrameStrategy;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import frc.robot.constants.Intake;

public class IntakeSubsystem extends SubsystemBase implements TelemetryNode {
  private final SparkMax m_motor;
  private final Encoder m_encoder;

  private boolean m_isStalled = false;
  private double m_voltage = 0.0;

  private enum State {
    IDLE(0),
    PREINTAKE(1),
    INTAKING(2),
    HAS_GAMEPIECE(3);

    public int value = 0;
    private Timer m_timer = new Timer();

    private State(int state) {
      value = state;
    }

    public int get() {
      return value;
    }

    public State advanceVelocity(double voltage, double velocity) {
      double absVoltage = Math.abs(voltage);
      if (absVoltage < 0.1) {
        return IDLE;
      }

      double expectedRPM = Math.abs(absVoltage * (Neo550MotorConstants.kFreeSpeedRpm / 12.0));
      switch (this) {
        case IDLE:
          if (absVoltage > 0.5) {
            var nextState = PREINTAKE;
            nextState.m_timer.restart();
            return nextState;
          }
          break;
        case PREINTAKE:
          if (m_timer.get() > 0.25) {
            return INTAKING;
          }
          break;
        case INTAKING:
          if (Math.abs(velocity) < (expectedRPM * 0.5)) {
            return HAS_GAMEPIECE;
          }
          break;
        case HAS_GAMEPIECE:
          if (Math.abs(velocity) > (expectedRPM * 0.75)) {
            var nextState = PREINTAKE;
            nextState.m_timer.restart();
            return PREINTAKE;
          }
          break;
        default:
          break;
      }

      return this;
    }

    public State advanceCurrent(double voltage, double current) {
      SmartDashboard.putNumber("Cube timer value", m_timer.get());
      double absVoltage = Math.abs(voltage);
      if (absVoltage < 0.1) {
        return IDLE;
      }

      switch (this) {
        case IDLE:
          if (absVoltage > 0.5) {
            var nextState = PREINTAKE;
            nextState.m_timer.restart();
            return nextState;
          }
          break;
        case PREINTAKE:
          if (m_timer.get() > 0.4) {
            return INTAKING;
          }
          break;
        case INTAKING:
          if (current > 15.0) {
            return HAS_GAMEPIECE;
          }
          break;
        case HAS_GAMEPIECE:
          if (current < 5.0) {
            var nextState = PREINTAKE;
            nextState.m_timer.restart();
            return PREINTAKE;
          }
          break;
        default:
          break;
      }

      return this;
    }
  }

  private State m_state = State.IDLE;
  private State m_currentBasedState = State.IDLE;

  public IntakeSubsystem() {
    m_motor =
        new SparkMax(Intake.kIntakeCANId).withInitializer(IntakeSubsystem::sparkMaxInitializer);
    m_encoder = m_motor.builtinEncoder();
  }

  @Override
  public void periodic() {
    m_motor.setVoltage(m_voltage);

    m_state = m_state.advanceVelocity(m_voltage, m_encoder.getVelocity());
    m_currentBasedState = m_currentBasedState.advanceCurrent(m_voltage, m_motor.getOutputCurrent());
  }

  public void setVoltage(double voltage) {
    m_voltage = voltage;
  }

  // Optimized really only for intaking, not after intaking
  // and resets to IDLE (false) when turning off the intake
  // even if it has a gamepiece.
  public boolean hasGamepiece() {
    return m_currentBasedState == State.HAS_GAMEPIECE;
  }

  // TODO: Move this to superstructure
  public Command setVoltageCommand(double voltage) {
    return new InstantCommand(() -> setVoltage(voltage), this);
  }

  public Command outtakeCommand(double power, double time) {
    SequentialCommandGroup commands =
        new SequentialCommandGroup(
            setVoltageCommand(power), new WaitCommand(time), setVoltageCommand(0.0));
    commands.addRequirements(this);
    return commands;
  }

  private static Boolean sparkMaxInitializer(CANSparkMax sparkMax, Boolean isInit) {
    int errors = 0;
    errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo500(sparkMax));
    errors += SparkMaxUtils.check(CANSparkMaxExtensions.setInverted(sparkMax, Intake.kMotorInvert));
    errors +=
        SparkMaxUtils.check(
            sparkMax.getEncoder().setVelocityConversionFactor(Intake.kVelocityFactor));
    errors += SparkMaxUtils.check(sparkMax.getEncoder().setAverageDepth(2));
    errors += SparkMaxUtils.check(sparkMax.getEncoder().setMeasurementPeriod(16));
    SparkMax.setFrameStrategy(sparkMax, FrameStrategy.kVelocity);
    return errors == 0;
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    builder.bindChild("SparkMax", m_motor);
    builder.bindChild("SparkMaxEncoder", m_encoder);
    builder.addIntegerProperty("State", () -> m_state.get(), null);
    builder.addIntegerProperty("CurrentBasedState", () -> m_currentBasedState.get(), null);
    builder.addBooleanProperty("HasGamepiece", this::hasGamepiece, null);
    builder.addDoubleProperty("VDemand", () -> m_voltage, null);
  }
}
