package frc.robot.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.swerve.SwerveDrive;
import frc.lib.telemetry.TelemetryBuilder;
import frc.robot.constants.Drivetrain;

public class SwerveDriveTuning extends AutonCommandBase {
  public SwerveDriveTuning(SwerveDrive swerve) {
    PathPlannerTrajectory trajectory = AutonUtils.loadTrajectory("Drive2M", 4.0, 6.5);
    // spotless:off
    addCommands(
        AutonUtils.flipFieldAndResetPose(swerve, trajectory),
        swerve.trajectoryFollowerCommand(
            trajectory,
            Drivetrain.kXController,
            Drivetrain.kYController,
            Drivetrain.kThetaController),
        new InstantCommand(() -> swerve.drive(0.0, 0.0, 0.0, false), swerve)
    );
    // spotless:on
  }

  @Override
  public void bind(TelemetryBuilder builder) {
    super.initSendable(builder);
  }
}
