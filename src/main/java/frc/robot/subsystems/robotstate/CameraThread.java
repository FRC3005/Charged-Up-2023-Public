package frc.robot.subsystems.robotstate;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Field;
import frc.robot.constants.Vision;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.LinkedBlockingQueue;
import org.photonvision.EstimatedRobotPose;
import org.tinylog.Logger;

public class CameraThread extends Thread {
  public static class CameraResult {
    public final String cameraName;
    public final Pose2d visionRobotPoseMeters;
    public final double timestampSeconds;
    public final Matrix<N3, N1> visionMeasurementStdDevs;

    public CameraResult(
        String camera,
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs) {
      this.cameraName = camera;
      this.visionRobotPoseMeters = visionRobotPoseMeters;
      this.timestampSeconds = timestampSeconds;
      this.visionMeasurementStdDevs = visionMeasurementStdDevs;
    }
  }

  private int m_bufferOverflowCnt = 0;
  private int m_queueCnt = 0;
  private static final int kQueueSizeMax = 10;
  private final LinkedBlockingQueue<CameraResult> m_resultQueue =
      new LinkedBlockingQueue<>(kQueueSizeMax);

  private List<Camera> m_cameras =
      Arrays.asList(
          new Camera(Vision.kCamName2, Vision.kCameraLocations.get(Vision.kCamName2)),
          new Camera(Vision.kCamName3, Vision.kCameraLocations.get(Vision.kCamName3)));

  private final Timer m_visionPoseTimer = new Timer();

  public CameraThread() {}

  private boolean isTargetAcceptable(EstimatedRobotPose robotPose, double tagDistance) {
    int tagCount = robotPose.targetsUsed.size();

    if (tagCount < 0) {
      return false;
    }

    // Stay on the field
    if (robotPose.estimatedPose.getX() < -Field.kBorderMargin
        || robotPose.estimatedPose.getX() > Field.kLength + Field.kBorderMargin
        || robotPose.estimatedPose.getY() < -Field.kBorderMargin
        || robotPose.estimatedPose.getY() > Field.kWidth + Field.kBorderMargin) {
      return false;
    }

    // Check that the pose height makes sense
    // TODO Check This, then remove it
    SmartDashboard.putNumber("Camera/Height", robotPose.estimatedPose.getZ());
    if (Math.abs(robotPose.estimatedPose.getZ()) > 0.75) {
      return false;
    }

    /*
     * This section needs tweaking based on performance
     */
    switch (tagCount) {
      case 0:
        return false;
      case 1:
        return tagDistance < 1.0 && robotPose.targetsUsed.get(0).getPoseAmbiguity() < 0.2;
      case 2:
        return tagDistance < 2.5;
      case 3:
        return tagDistance < 3.5;
      default:
        return tagDistance < 4.0;
    }
  }

  @Override
  public void run() {
    Timer loopTimer = new Timer();
    while (true) {
      loopTimer.restart();
      runCameras();
      double loopTime = loopTimer.get();
      if (loopTime < 0.02) {
        if (loopTime < 0.0) {
          loopTime = 0.0;
        }
        Timer.delay(0.02 - loopTime);
      }
    }
  }

  public CameraResult getNextResultOrNull() {
    return m_resultQueue.poll();
  }

  private void runCameras() {
    for (var camera : m_cameras) {
      // getEstimatedPosition is only used for certain multi-target strategies,
      // multi-target PNP is not one of them. See Camera.java
      m_visionPoseTimer.restart();
      Optional<EstimatedRobotPose> res = camera.getEstimatedGlobalPose();
      SmartDashboard.putNumber("Camera/Get Pose Timer", m_visionPoseTimer.get());

      // Possible optimization, currently disabled. Only use this if multiple tags are seen.
      if (res.isPresent()) {
        EstimatedRobotPose robotPose = res.get();
        Pose2d pose2d = robotPose.estimatedPose.toPose2d();

        // Find closest tag distance (meters)
        double distance = 100;
        for (var target : robotPose.targetsUsed) {
          double t_distance =
              target.getBestCameraToTarget().getTranslation().getDistance(new Translation3d());
          if (t_distance < distance) {
            distance = t_distance;
          }
        }
        SmartDashboard.putNumber("Camera/Camera Distance " + camera.getName(), distance);

        // Mechanical Advantages does this:
        // double xyStdDev = 0.01 * Math.pow(distance, 2.0) / robotPose.targetsUsed.size();
        // double thetaStdDev = 0.01 * Math.pow(distance, 2.0) / robotPose.targetsUsed.size();

        double xyStdDev = distance / 20.0;
        double thetaStdDev = 1000; // Units.degreesToRadians(10);

        if (isTargetAcceptable(robotPose, distance)) {
          CameraResult result =
              new CameraResult(
                  camera.getName(),
                  pose2d,
                  robotPose.timestampSeconds,
                  VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
          try {
            if (!m_resultQueue.offer(result)) {
              m_resultQueue.poll();
              m_resultQueue.put(result);
              m_bufferOverflowCnt += 1;
              SmartDashboard.putNumber("Camera/BufferOverflow", m_bufferOverflowCnt);
            }
            SmartDashboard.putNumber("Camera/QueueCnt", m_queueCnt++);
          } catch (InterruptedException e) {
            Logger.tag("Camera Thread")
                .error("Unable to push results for {}: {}", camera.getName(), e);
          }
        }
      }
    }
  }
}
