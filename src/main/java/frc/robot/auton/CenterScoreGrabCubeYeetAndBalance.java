package frc.robot.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import frc.lib.vendor.sensor.ADIS16470;
import frc.robot.subsystems.drive.DriveSubsystem;

public class CenterScoreGrabCubeYeetAndBalance extends AutonCommandBase {

  DriveSubsystem driveSubsystem;

  public CenterScoreGrabCubeYeetAndBalance(DriveSubsystem swerve, ADIS16470 imu) {
    driveSubsystem = swerve;
    ClimbRampCommand climbCommand = new ClimbRampCommand(swerve, imu);
    PathPlannerTrajectory trajectory = AutonUtils.loadTrajectory("CenterGrabOne", 1.8, 5.5);

    var events = new AutonEvents();

    addCommands(
        AutonUtils.flipFieldAndResetPose(swerve, trajectory),
        events.scoreConeMid(),
        new FollowPathWithEvents(
            driveSubsystem.trajectoryFollowerCommand(trajectory),
            trajectory.getMarkers(),
            events.get()),
        climbCommand.balance(),
        events.chuckCube());
  }
}
