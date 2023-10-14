package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.geometry.Point2d;
import frc.lib.odometry.Carpet;
import frc.lib.odometry.CarpetZone;

public final class Field {
  public static final double kLength = Units.inchesToMeters(54 * 12);
  public static final double kMaxX = kLength;
  public static final double kWidth = Units.inchesToMeters(27 * 12);
  public static final double kMaxY = kWidth;
  public static final double kBorderMargin = 0.5;

  public static final class HomeField {
    public static final double kMidLineY = 3.454;
    public static final double kMidLineX = Field.kLength / 2.0 + 2.295;

    // Start at the red loading station. Look at the carpet. If the nap goes
    // down
    // towards blue, the angle is 0 for the first number. (If you lightly push the carpet forward
    // towards
    // blue, it has less resistance compared to pulling back towards red). Otherwise if its laying
    // towards
    // red, this number is 180 degrees. Range should be [-180, 180] (CCW+)
    // The second number is the same experiment, but standing on the red side on the bump side
    public static final Carpet kMercuryCarpet =
        Carpet.centerLineCarpet(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0));
    public static final Carpet kApolloCarpet =
        Carpet.centerLineCarpet(Rotation2d.fromDegrees(180.0), Rotation2d.fromDegrees(180.0));
    public static final Carpet kGalileo =
        Carpet.centerLineCarpet(Rotation2d.fromDegrees(180.0), Rotation2d.fromDegrees(180.0));

    public static final Carpet kHomeCarpet =
        new Carpet(
            // Red side loading
            new CarpetZone(
                new Point2d(-1, -1),
                new Point2d(kMidLineX, kMidLineY),
                Rotation2d.fromDegrees(180)),
            // Red side wall
            new CarpetZone(
                new Point2d(-1, kMidLineY), new Point2d(kMidLineX, 100), new Rotation2d()),
            // Blue side loading
            new CarpetZone(
                new Point2d(kMidLineX, -1),
                new Point2d(100, kMidLineY),
                Rotation2d.fromDegrees(180)),
            // Blue side wall
            new CarpetZone(
                new Point2d(kMidLineX, kMidLineY), new Point2d(100, 100), new Rotation2d()));

    // Finally, set the actual competition carpet
    public static final Carpet kCompetitionCarpet = kGalileo;
  }
}
