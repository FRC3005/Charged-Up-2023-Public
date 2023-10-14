package frc.robot.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ScoreAndDriveForward extends AutonCommandBase {

  DriveSubsystem driveSubsystem;

  public ScoreAndDriveForward(DriveSubsystem swerve) {
    driveSubsystem = swerve;
    PathPlannerTrajectory trajectory = AutonUtils.loadTrajectory("LeaveZone", 2.0, 6.0);

    var events = new AutonEvents();

    addCommands(
        AutonUtils.flipFieldAndResetPose(swerve, trajectory),
        events.scoreConeMid(),
        new FollowPathWithEvents(
            driveSubsystem.trajectoryFollowerCommand(trajectory),
            trajectory.getMarkers(),
            events.get()),
        new InstantCommand(() -> swerve.drive(0.0, 0.0, 0.0, false), swerve));
  }
}
