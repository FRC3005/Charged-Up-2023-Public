/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems.robotstate;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.tinylog.Logger;

public class Camera {
  private PhotonCamera m_photonCamera;
  private PhotonPoseEstimator m_photonPoseEstimator;
  private static AprilTagFieldLayout m_fieldLayout = null;

  /**
   * Call this once the alliance color is valid. See here for good discussions on when that is.
   * https://www.chiefdelphi.com/t/getalliance-always-returning-red/425782
   */
  public static void setFieldOrigin(Alliance allianceColor) {
    OriginPosition originPosition =
        allianceColor == Alliance.Blue
            ? OriginPosition.kBlueAllianceWallRightSide
            : OriginPosition.kRedAllianceWallRightSide;

    if (m_fieldLayout != null) {
      Logger.tag("Camera")
          .info(
              "Setting field origin to {}, alliance color {}",
              originPosition.toString(),
              allianceColor.toString());
      m_fieldLayout.setOrigin(originPosition);
    }
  }

  public Camera(String name, Transform3d robotToCamera) {
    m_photonCamera = new PhotonCamera(name);

    if (m_fieldLayout == null) {
      try {
        // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the
        // field.
        // Load Deploy/competition-field-layout.json first if its there, otherwise load default
        // This allows us to tweak the actual field locations if needed.
        try {
          m_fieldLayout =
              new AprilTagFieldLayout(
                  Filesystem.getDeployDirectory().getAbsolutePath()
                      + "competition-field-layout.json");
          Logger.tag("Camera").info("Using custom field layout");
        } catch (Exception e) {
          m_fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
          Logger.tag("Camera").info("Using built-in field layout");
        }
      } catch (IOException e) {
        m_fieldLayout = null;
        Logger.tag("Camera").error("Failed to load AprilTagFieldLayout\r\n{}", e.toString());
      }
    }

    if (m_fieldLayout == null) {
      m_photonPoseEstimator = null;
    } else {
      // Create pose estimator
      m_photonPoseEstimator =
          new PhotonPoseEstimator(
              m_fieldLayout, PoseStrategy.MULTI_TAG_PNP, m_photonCamera, robotToCamera);
      // Ideally this should be reference pose fallback, but threading makes it more complex
      m_photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT);
    }
  }

  public String getName() {
    return m_photonCamera.getName();
  }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
   *     the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() { // Pose2d prevEstimatedRobotPose) {
    if (m_photonPoseEstimator == null) {
      // The field layout failed to load, so we cannot estimate poses.
      return Optional.empty();
    }

    // m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return m_photonPoseEstimator.update();
  }
}
