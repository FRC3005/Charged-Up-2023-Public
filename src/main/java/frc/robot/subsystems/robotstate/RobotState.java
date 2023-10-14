package frc.robot.subsystems.robotstate;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.odometry.CarpetTransform;
import frc.lib.odometry.SwerveDriveCarpetPoseEstimator;
import frc.lib.swerve.SwerveDrive;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;
import frc.lib.vendor.sensor.ADIS16470;
import frc.robot.constants.Drivetrain;
import frc.robot.constants.Field;
import frc.robot.subsystems.robotstate.CameraThread.CameraResult;
import java.util.HashMap;
import org.tinylog.Logger;

public class RobotState extends SubsystemBase implements TelemetryNode {

  private static RobotState m_instance = null;
  private final SwerveDrive m_swerve;

  private final ADIS16470 m_gyro;

  // Use specific numbers well off the field to make it obvious this is on purpose
  private static final Pose2d kInvalidPose = new Pose2d(-1337, 3005, new Rotation2d());

  private final Field2d m_field = new Field2d();
  private boolean m_disableVision = false;

  private final SwerveDriveCarpetPoseEstimator m_poseEstimator;

  private Alliance m_allianceColor = Alliance.Invalid;

  private final CameraThread m_cameraThread;

  private HashMap<String, Double> m_tagPoseTimestamp = new HashMap<>();

  private final CarpetTransform m_transform =
      new CarpetTransform(Drivetrain.kCarpetLookup, Field.HomeField.kCompetitionCarpet);

  public static RobotState getInstance(SwerveDrive swerve, ADIS16470 gyro) {
    if (m_instance == null) {
      m_instance = new RobotState(swerve, gyro);
    }
    return m_instance;
  }

  public static RobotState getInstance() {
    return m_instance;
  }

  private RobotState(SwerveDrive swerve, ADIS16470 gyro) {
    m_swerve = swerve;
    m_gyro = gyro;
    m_poseEstimator =
        new SwerveDriveCarpetPoseEstimator(
            Drivetrain.kDriveCarpetKinematics,
            new Rotation2d(),
            m_swerve.getModulePositions(),
            m_transform,
            VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(1)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(40)));
    m_cameraThread = new CameraThread();
    m_cameraThread.start();
  }

  public void start() {
    m_cameraThread.start();
  }

  /**
   * Get the pose of the robot using fused vision and wheel odometry measurements.
   *
   * @return pose of the robot based on vision and odometry measurements.
   */
  public Pose2d getPoseEstimate() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Reset the pose estimation to a fixed pose.
   *
   * @param resetPose pose to set the estimator to.
   */
  public void resetPoseEstimate(Pose2d resetPose) {
    m_poseEstimator.resetPosition(m_gyro.getRotation2d(), m_swerve.getModulePositions(), resetPose);
  }

  public void disableVision() {
    m_disableVision = true;
  }

  public void enableVision() {
    m_disableVision = false;
  }

  public boolean isVisionEnabled() {
    return !m_disableVision;
  }

  @Override
  public void periodic() {
    // TODO: Get timestamps for swerve odometry or otherwise correct for it
    m_poseEstimator.update(m_gyro.getRotation2d(), m_swerve.getModulePositions());

    updateAllianceColor();

    // Get results from camera thread and update robot pose
    for (CameraResult result = m_cameraThread.getNextResultOrNull();
        result != null;
        result = m_cameraThread.getNextResultOrNull()) {

      if (m_disableVision) {
        continue;
      }

      m_poseEstimator.addVisionMeasurement(
          result.visionRobotPoseMeters, result.timestampSeconds, result.visionMeasurementStdDevs);
      m_field.getObject(result.cameraName).setPose(result.visionRobotPoseMeters);

      m_tagPoseTimestamp.put(result.cameraName, result.timestampSeconds);
    }

    // Invalidate old field poses for telemetry
    for (var cameraPoseTimestamp : m_tagPoseTimestamp.entrySet()) {
      double timeNow = Timer.getFPGATimestamp();

      if (timeNow - cameraPoseTimestamp.getValue() > 1.0) {
        m_field.getObject(cameraPoseTimestamp.getKey()).setPose(kInvalidPose);
      }
    }

    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
  }

  public void resetSwerveOdometryToEstimate() {
    m_swerve.resetOdometry(m_poseEstimator.getEstimatedPosition());
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    builder.bindSendableChild("Field", m_field);
    builder.addBooleanProperty(
        "Reset Odometry to Vision", () -> false, (val) -> resetSwerveOdometryToEstimate());
  }

  public void testModePeriodic() {
    periodic();
  }

  private void updateAllianceColor() {
    Alliance alliance = DriverStation.getAlliance();
    if (alliance == Alliance.Invalid) {
      m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
      return;
    }

    if (m_allianceColor != alliance) {
      if (!DriverStation.isDSAttached()) {
        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
        return;
      }

      Logger.tag("Robot State")
          .info("Alliance color set, was {}, setting to {}", m_allianceColor, alliance);
      m_allianceColor = alliance;
      Camera.setFieldOrigin(m_allianceColor);
    }
  }
}
