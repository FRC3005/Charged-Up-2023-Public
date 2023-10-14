package frc.robot.constants;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.Map;

public final class Vision {
  public static final String kCamName1 = "gordon";
  public static final String kCamName2 = "percy";
  public static final String kCamName3 = "james";

  public static final Map<String, Transform3d> kCameraLocations =
      Map.of(
          kCamName1,
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(-8.759),
                      Units.inchesToMeters(5.500),
                      Units.inchesToMeters(50.753)),
                  new Rotation3d(new Quaternion(0.9962, 0, 0.0872, 0))),
          kCamName2,
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(-9.658),
                      Units.inchesToMeters(7.278),
                      Units.inchesToMeters(42.229)),
                  new Rotation3d(new Quaternion(0.29, -0.247, -0.078, 0.921))),
          kCamName3,
              new Transform3d(
                  new Translation3d(
                      Units.inchesToMeters(-9.658),
                      Units.inchesToMeters(5.722),
                      Units.inchesToMeters(42.229)),
                  new Rotation3d(new Quaternion(-0.29, -0.247, 0.078, 0.921))));
}
