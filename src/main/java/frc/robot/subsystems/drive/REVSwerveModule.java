package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxExtensions;
import com.revrobotics.CANSparkMaxHandle;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMaxSim1;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxDutyCycleSensorSim;
import com.revrobotics.jni.CANSparkMaxJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.lib.controller.Controller;
import frc.lib.controller.SimpleMotorFeedforward;
import frc.lib.electromechanical.Encoder;
import frc.lib.sim.SwerveModuleSim;
import frc.lib.swerve.SwerveModule;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.vendor.motorcontroller.SparkMax;
import frc.lib.vendor.motorcontroller.SparkMaxDutyCycleSensor;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import frc.robot.constants.Drivetrain;
import frc.robot.constants.RobotConstants;

public class REVSwerveModule implements SwerveModule, AutoCloseable {
  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;
  private final Controller m_turningController;
  private final Controller m_driveController;

  private final Encoder m_turningNeoEncoder;
  private final SparkMaxDutyCycleSensor m_turningEncoder;
  private final Encoder m_driveNeoEncoder;

  private final SimpleMotorFeedforward m_driveFeedforward;
  private final double m_turningOffset;

  // Simulation Functions
  private final CANSparkMaxSim1 m_turningMotorSim;
  private final CANSparkMaxSim1 m_driveMotorSim;
  private final SwerveModuleSim m_swerveSim;
  protected final SparkMaxDutyCycleSensorSim m_turningSensorSim;
  private SimpleMotorFeedforward m_turningFeedforward =
      new SimpleMotorFeedforward(Drivetrain.kTurningFeedforward);

  private boolean m_testEnable = false;

  // Desired state of the Spark Max itself after accounting for offset
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Create a REV Swerve Module. This module expects a REV Swerve, with a NEO drive and NEO 500 for
   * turning, with a through bore encoder, plugged into the top of the Spark Max. Each module has
   * its offset stored in flash from the hardware client.
   *
   * @param driveCanId CAN Id for the drive Spark Max
   * @param turningCanId CAN Id for the turning Spark Max, with through bore encoder
   * @param turningOffset Offset from 0 in radians which the module is set for. For a typical 4
   *     corner swerve in the range [-pi, pi] this will be one of 0, pi/2, pi, -pi/2.
   */
  public REVSwerveModule(int driveCanId, int turningCanId, double turningOffset) {
    m_driveMotor =
        new SparkMax(driveCanId, MotorType.kBrushless)
            .withInitializer(REVSwerveModule::driveMotorConfig);
    m_turningMotor =
        new SparkMax(turningCanId, MotorType.kBrushless)
            .withInitializer(REVSwerveModule::turningMotorConfig);

    m_turningNeoEncoder = m_turningMotor.builtinEncoder();
    m_driveNeoEncoder = m_driveMotor.builtinEncoder();
    m_driveNeoEncoder.setPosition(0.0);
    m_turningEncoder = new SparkMaxDutyCycleSensor(m_turningMotor);
    m_turningOffset = turningOffset;

    m_driveFeedforward = new SimpleMotorFeedforward(Drivetrain.kDriveFeedforward);

    m_driveController = m_driveMotor.velocityController(Drivetrain.kDriveMotorPIDGains);
    m_turningController =
        m_turningMotor.positionController(
            Drivetrain.kTurningMotorPIDGains, m_turningEncoder.getSensor());

    m_turningController.enableContinuousInput(-Math.PI, Math.PI);

    // Simulation init
    m_turningMotorSim = new CANSparkMaxSim1(m_turningMotor);
    m_driveMotorSim = new CANSparkMaxSim1(m_driveMotor);
    m_turningSensorSim = new SparkMaxDutyCycleSensorSim(m_turningEncoder.getSensor());
    m_swerveSim =
        new SwerveModuleSim(
            DCMotor.getNEO(1).withReduction(Drivetrain.kDriveMotorReduction),
            Drivetrain.kDriveFeedforward.kv,
            Drivetrain.kDriveFeedforward.ka,
            DCMotor.getNeo550(1).withReduction(Drivetrain.kTurningModuleGearRatio),
            Drivetrain.kTurningFeedforward.kv
                * 500, // Not sure why * 500, but that does stabilize it
            Drivetrain.kTurningFeedforward.ka * 500);

    // Set the position to the absolute values
    resetEncoders();

    // Reset the angle to current angle
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
  }

  /**
   * Configure function for drive motor. Configures the spark max conversion factors, inversion, and
   * current limits based on java, plus it sets idle mode, output range, and periodic frame rates.
   * This is fed directly into the SparkMax constructions using the withInitializer() method.
   *
   * @param sparkMax the spark max being configured
   * @param isInit true if this is the first initialization on robot power on, false generally means
   *     a reset occured.
   * @return true if configuration succeded
   */
  private static Boolean driveMotorConfig(CANSparkMax sparkMax, Boolean isInit) {
    int errors = 0;
    RelativeEncoder enc = sparkMax.getEncoder();

    // Convert 'rotations' to 'meters'
    errors +=
        SparkMaxUtils.check(
            enc.setPositionConversionFactor(Drivetrain.kDriveEncoderPositionFactor));

    // Convert 'RPM' to 'meters per second'
    errors +=
        SparkMaxUtils.check(
            enc.setVelocityConversionFactor(Drivetrain.kDriveEncoderVelocityFactor));

    // Set inversion
    errors +=
        SparkMaxUtils.check(
            CANSparkMaxExtensions.setInverted(sparkMax, Drivetrain.kDriveMotorInvert));

    // Configure velocity decoding, 16ms filter, 2 taps
    errors += SparkMaxUtils.check(enc.setMeasurementPeriod(16));
    errors += SparkMaxUtils.check(enc.setAverageDepth(2));

    errors += SparkMaxUtils.check(sparkMax.getPIDController().setOutputRange(-1, 1));
    errors += SparkMaxUtils.check(sparkMax.setIdleMode(IdleMode.kCoast));
    errors +=
        SparkMaxUtils.check(sparkMax.setSmartCurrentLimit(Drivetrain.kDriveMotorCurrentLimit));

    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(sparkMax, PeriodicFrame.kStatus0, 100));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(sparkMax, PeriodicFrame.kStatus1, 15));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(sparkMax, PeriodicFrame.kStatus2, 15));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(sparkMax, PeriodicFrame.kStatus3, 200));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(sparkMax, PeriodicFrame.kStatus5, 10000));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(sparkMax, PeriodicFrame.kStatus6, 10000));

    return errors == 0;
  }

  /**
   * Configure function for turning motor. Configures the spark max conversion factors, inversion,
   * and current limits based on java, plus it sets idle mode, output range, and periodic frame
   * rates. This is fed directly into the SparkMax constructions using the withInitializer() method.
   *
   * @param sparkMax the spark max being configured
   * @param isInit true if this is the first initialization on robot power on, false generally means
   *     a reset occured.
   * @return true if configuration succeded
   */
  private static Boolean turningMotorConfig(CANSparkMax sparkMax, Boolean isInit) {
    int errors = 0;
    CANSparkMaxHandle m_handle = new CANSparkMaxHandle(sparkMax);
    RelativeEncoder enc = sparkMax.getEncoder();
    SparkMaxAbsoluteEncoder dcEncoder = sparkMax.getAbsoluteEncoder(Type.kDutyCycle);

    // Convert 'rotations' to 'meters'
    errors +=
        SparkMaxUtils.check(
            enc.setPositionConversionFactor(Drivetrain.kTurningEncoderPositionFactor));

    // Convert 'RPM' to 'meters per second'
    errors +=
        SparkMaxUtils.check(
            enc.setVelocityConversionFactor(Drivetrain.kTurningEncoderVelocityFactor));

    // Convert rotations to radians
    // Enable center aligned mode to go [-pi, pi]
    errors +=
        SparkMaxUtils.check(
            REVLibError.fromInt(
                CANSparkMaxJNI.c_SparkMax_SetParameterBool(m_handle.handle, 152, true)));
    errors +=
        SparkMaxUtils.check(
            dcEncoder.setPositionConversionFactor(Math.PI + Units.degreesToRadians((0.35122 / 2))));
    errors += SparkMaxUtils.check(dcEncoder.setVelocityConversionFactor(Math.PI));

    // Phase of the duty cycle encoder is opposite of the motor
    errors += SparkMaxUtils.check(dcEncoder.setInverted(!Drivetrain.kTurningMotorInvert));

    // Set inversion
    errors +=
        SparkMaxUtils.check(
            CANSparkMaxExtensions.setInverted(sparkMax, Drivetrain.kTurningMotorInvert));

    errors += SparkMaxUtils.check(sparkMax.getPIDController().setOutputRange(-1, 1));
    errors += SparkMaxUtils.check(sparkMax.setIdleMode(IdleMode.kBrake));
    errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo500(sparkMax));

    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(sparkMax, PeriodicFrame.kStatus0, 100));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(sparkMax, PeriodicFrame.kStatus1, 25));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(sparkMax, PeriodicFrame.kStatus2, 50));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(sparkMax, PeriodicFrame.kStatus3, 200));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(sparkMax, PeriodicFrame.kStatus5, 8));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.setPeriodicFramePeriod(sparkMax, PeriodicFrame.kStatus6, 50));

    return errors == 0;
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    if (!RobotConstants.kReducedTelemetryMode) {
      builder.bindChild("Drive Controller", m_driveController);
      builder.bindChild("Turn Controller", m_turningController);
      builder.addDoubleProperty(
          "Velocity Setpoint",
          () -> m_desiredState.speedMetersPerSecond,
          val -> setDesiredState(new SwerveModuleState(val, m_desiredState.angle)));
      builder.addDoubleProperty(
          "Turning Setpoint",
          () -> m_desiredState.angle.getRadians(),
          (val) ->
              setDesiredState(
                  new SwerveModuleState(m_desiredState.speedMetersPerSecond, new Rotation2d(val))));
      builder.addDoubleProperty(
          "Drive kS", () -> m_driveFeedforward.ks, (k) -> m_driveFeedforward.ks = k);
      builder.addDoubleProperty(
          "Drive kA", () -> m_driveFeedforward.ka, (k) -> m_driveFeedforward.ka = k);
      builder.addDoubleProperty(
          "Drive kV", () -> m_driveFeedforward.kv, (k) -> m_driveFeedforward.kv = k);

      builder.bindChild("TurningEncoder", m_turningNeoEncoder);
      builder.bindChild("DriveEncoder", m_driveNeoEncoder);
      builder.bindChild("TurningAbsolute", m_turningEncoder);
      builder.bindChild("TurningEncoder", m_turningNeoEncoder);
      builder.addBooleanProperty(
          "Testmode Enable", () -> m_testEnable, (val) -> m_testEnable = val);
      builder.addDoubleProperty("Drive Current", () -> m_driveMotor.getOutputCurrent(), null);
      builder.addDoubleProperty("Turning Current", () -> m_turningMotor.getOutputCurrent(), null);
      builder.bindChild("Turning Motor", m_turningMotor);
      builder.bindChild("Drive Motor", m_driveMotor);
    }
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveNeoEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition()).plus(new Rotation2d(m_turningOffset)));
  }

  @Override
  public SwerveModuleState getActualState() {
    return new SwerveModuleState(
        m_driveNeoEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Set the desired module state. This includes (robot centric) turing theta in radians, and speed
   * in meters per second.
   *
   * @param desiredState the desired state to set the module to.
   */
  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    // Everything from this point on is relative to the sparkmax (i.e. after offset)
    desiredState.angle = desiredState.angle.minus(Rotation2d.fromRadians(m_turningOffset));

    // desired state must be withing [-pi, pi]
    if (Math.abs(desiredState.angle.getRadians()) > Math.PI) {
      org.tinylog.Logger.tag("Swerve Module " + m_turningMotor.getDeviceId())
          .error("Desired state: {} degrees", desiredState.angle.getRadians());

      desiredState.angle = new Rotation2d(MathUtil.angleModulus(desiredState.angle.getRadians()));
    }

    // Optimize the reference state to avoid spinning further than 90 degrees.
    // Important: Use spark max sensor directly, not wrapped sensor
    desiredState =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getPosition()));

    m_desiredState = desiredState;
  }

  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  @Override
  public void holdHeading(double speedMetersPerSecond) {
    m_desiredState.speedMetersPerSecond = speedMetersPerSecond;
  }

  @Override
  public void periodic() {
    // Calculate the turning motor output from the turning PID controller.
    m_driveController.setReference(
        m_desiredState.speedMetersPerSecond,
        m_driveFeedforward.calculate(m_desiredState.speedMetersPerSecond));

    m_turningController.setReference(m_desiredState.angle.getRadians());
    // m_desiredState.angle.getRadians(),
    // m_turningEncoder.getPosition(),
    // (setpoint) -> m_turningFeedforward.calculate(setpoint.position, setpoint.velocity));
  }

  @Override
  public void testPeriodic() {
    if (RobotBase.isSimulation()) {
      simulationPeriodic();
    }

    if (!m_testEnable) {
      m_turningMotor.setVoltage(0.0);
    } else {
      periodic();
    }
  }

  @Override
  public void simulationPeriodic() {
    double vbus = RobotController.getBatteryVoltage();
    m_turningMotorSim.enable();
    m_driveMotorSim.enable();

    m_turningMotorSim.iterate(m_swerveSim.getTurningMotorVelocityRPM(), vbus, 0.02);
    m_driveMotorSim.iterate(m_swerveSim.getDriveMotorVelocityRPM(), vbus, 0.02);

    m_swerveSim.iterate(
        m_driveMotorSim.getAppliedOutput() * vbus,
        m_turningMotorSim.getAppliedOutput() * vbus,
        0.02);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_swerveSim.getCurrentDrawAmps()));

    m_turningMotorSim.setMotorCurrent(m_swerveSim.getTurningMotorCurrentDrawAmps());
    m_driveMotorSim.setMotorCurrent(m_swerveSim.getDriveMotorCurrentDrawAmps());
  }

  @Override
  public void resetEncoders() {
    m_driveNeoEncoder.setPosition(0);
    m_turningNeoEncoder.setPosition(
        new Rotation2d(m_turningEncoder.getPosition())
            .plus(new Rotation2d(m_turningOffset))
            .getRadians());
  }

  @Override
  public double getDriveDistanceMeters() {
    return m_driveNeoEncoder.getPosition();
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveNeoEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition()).plus(new Rotation2d(m_turningOffset)));
  }

  @Override
  public void close() {
    m_turningMotor.close();
    m_driveMotor.close();
  }

  protected void simSetTurningMotorPosition(Rotation2d theta) {
    m_turningSensorSim.setPosition(theta.getRadians());
  }
}
