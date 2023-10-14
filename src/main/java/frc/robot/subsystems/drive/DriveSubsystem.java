package frc.robot.subsystems.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.swerve.SwerveDrive;
import frc.lib.swerve.SwerveModule;
import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.vendor.sensor.ADIS16470;
import frc.robot.constants.Drivetrain;
import java.util.function.Supplier;

public class DriveSubsystem extends SwerveDrive {
  public Command trajectoryFollowerCommand(PathPlannerTrajectory trajectory) {
    Drivetrain.kThetaController.enableContinuousInput(-Math.PI, Math.PI);
    return trajectoryFollowerCommand(
        trajectory, Drivetrain.kXController, Drivetrain.kYController, Drivetrain.kThetaController);
  }

  public Command trajectoryFollowerCommand(
      PathPlannerTrajectory trajectory, Supplier<Pose2d> poseSupplier) {
    Drivetrain.kThetaController.enableContinuousInput(-Math.PI, Math.PI);
    return trajectoryFollowerCommand(
        trajectory,
        poseSupplier,
        Drivetrain.kXController,
        Drivetrain.kYController,
        Drivetrain.kThetaController);
  }

  static final SwerveModule frontLeft =
      new REVSwerveModule(
          Drivetrain.kFrontLeftDriveCanId, Drivetrain.kFrontLeftTurningCanId, Math.PI / 2.0);

  static final SwerveModule frontRight =
      new REVSwerveModule(Drivetrain.kFrontRightDriveCanId, Drivetrain.kFrontRightTurningCanId, 0);

  static final SwerveModule rearLeft =
      new REVSwerveModule(
          Drivetrain.kRearLeftDriveCanId, Drivetrain.kRearLeftTurningCanId, Math.PI);

  static final SwerveModule rearRight =
      new REVSwerveModule(
          Drivetrain.kRearRightDriveCanId, Drivetrain.kRearRightTurningCanId, 3.0 * Math.PI / 2.0);

  public DriveSubsystem(ADIS16470 gyro) {
    super(
        frontLeft,
        frontRight,
        rearLeft,
        rearRight,
        Drivetrain.kDriveKinematics,
        gyro,
        Drivetrain.kMaxDriveSpeed);
    // Logger.tag("Swerve Drive").warn("Reset calibration time back to longer for comp");
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    super.bind(builder);
    builder.bindSendableChild("X Controller", Drivetrain.kXController);
    builder.bindSendableChild("Y Controller", Drivetrain.kYController);
    builder.bindSendableChild("Theta Controller", Drivetrain.kThetaController);
  }
}
