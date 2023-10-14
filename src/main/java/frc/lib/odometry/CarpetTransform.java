package frc.lib.odometry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.InterpolatingTreeMap;
import java.util.List;

public class CarpetTransform {
  /** Key = theta (radians), Value = percent of effective wheel radius change (1.0 == 100%) */
  private final InterpolatingTreeMap<Double, Double> m_lookup = new InterpolatingTreeMap<>();

  private final Carpet m_carpet;

  public CarpetTransform(List<Pair<Rotation2d, Double>> points, Carpet carpet) {
    double min = MathUtil.angleModulus(points.get(0).getFirst().getRadians());
    double max = min;
    for (var pair : points) {
      double angle = MathUtil.angleModulus(pair.getFirst().getRadians());

      if (angle < min) {
        min = angle;
      } else if (angle > max) {
        max = angle;
      }

      m_lookup.put(angle, pair.getSecond());
    }

    m_carpet = carpet;

    // Make sure the interpolation wraps
    if (min == -Math.PI && max == Math.PI) {
      // TODO: Should this throw an error if the two points are too far from eachother?
      return;
    }

    double minVal = m_lookup.get(min);
    double maxVal = m_lookup.get(max);
    double maxDelta = (Math.PI - max);
    double minDelta = (Math.PI - Math.abs(min));
    double midpoint = minDelta / (minDelta + maxDelta);
    double wrapValue = MathUtil.interpolate(minVal, maxVal, midpoint);

    if (min > -Math.PI) {
      m_lookup.put(-Math.PI, wrapValue);
    }
    if (max < Math.PI) {
      m_lookup.put(Math.PI, wrapValue);
    }
  }

  public double getTransform(Rotation2d angle) {
    double angleRads = MathUtil.angleModulus(angle.getRadians());
    return m_lookup.get(angleRads);
  }

  public double getTransform(Pose2d pose) {
    Rotation2d carpetDirection = m_carpet.carpetDirection(pose.getX(), pose.getY());
    Rotation2d correctedDirection = pose.getRotation().minus(carpetDirection);
    return m_lookup.get(MathUtil.angleModulus(correctedDirection.getRadians()));
  }
}
