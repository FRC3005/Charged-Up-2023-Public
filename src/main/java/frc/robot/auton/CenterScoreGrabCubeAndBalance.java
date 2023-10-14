package frc.robot.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import frc.lib.vendor.sensor.ADIS16470;
import frc.robot.subsystems.drive.DriveSubsystem;

public class CenterScoreGrabCubeAndBalance extends AutonCommandBase {

  DriveSubsystem driveSubsystem;

  public CenterScoreGrabCubeAndBalance(DriveSubsystem swerve, ADIS16470 imu, boolean grabCube) {
    super(grabCube ? "CenterScoreOverGrabCubeBalance" : "CenterScoreOverBalance");
    driveSubsystem = swerve;
    ClimbRampCommand climbCommand = new ClimbRampCommand(swerve, imu);
    PathPlannerTrajectory trajectory;

    if (grabCube) {
      trajectory = AutonUtils.loadTrajectory("CenterGrabOne", 1.8, 3.5);
    } else {
      trajectory = AutonUtils.loadTrajectory("CenterOverAndBalance", 1.5, 3.0);
    }

    var events = new AutonEvents();

    addCommands(
        AutonUtils.flipFieldAndResetPose(swerve, trajectory),
        events.scoreConeHighStart(),
        events.telescopeHome(),
        new FollowPathWithEvents(
            driveSubsystem.trajectoryFollowerCommand(trajectory),
            trajectory.getMarkers(),
            events.get()),
        climbCommand.balance());
  }
}
