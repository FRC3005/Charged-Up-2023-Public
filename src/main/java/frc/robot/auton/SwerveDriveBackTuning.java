package frc.robot.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.swerve.SwerveDrive;
import frc.lib.telemetry.TelemetryBuilder;
import frc.robot.constants.Drivetrain;

public class SwerveDriveBackTuning extends AutonCommandBase {
  public SwerveDriveBackTuning(SwerveDrive swerve) {
    PathPlannerTrajectory trajectory = AutonUtils.loadTrajectory("Drive2M", 4.0, 4.0, true);
    // spotless:off
    addCommands(
        AutonUtils.flipFieldAndResetPose(swerve, trajectory),
        swerve.trajectoryFollowerCommand(
            trajectory,
            Drivetrain.kXController,
            Drivetrain.kYController,
            new PIDController(0.0, 0.0, 0.0)),
        new InstantCommand(() -> swerve.drive(0.0, 0.0, 0.0, false), swerve)
    );
    // spotless:on
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    super.initSendable(builder);
  }
}
