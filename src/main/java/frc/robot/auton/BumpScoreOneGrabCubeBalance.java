package frc.robot.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import frc.lib.vendor.sensor.ADIS16470;
import frc.robot.subsystems.drive.DriveSubsystem;

public class BumpScoreOneGrabCubeBalance extends AutonCommandBase {

  DriveSubsystem driveSubsystem;

  public BumpScoreOneGrabCubeBalance(DriveSubsystem swerve, ADIS16470 imu, boolean doBalance) {
    super(doBalance ? "BumpScoreOneGrabCubeBalance" : "BumpScoreOneGrabCube");
    driveSubsystem = swerve;
    ClimbRampCommand climbCommand = new ClimbRampCommand(swerve, imu);
    PathPlannerTrajectory trajectory = AutonUtils.loadTrajectory("BumpToBalancePlusOne", 1.4, 5.0);

    var events = new AutonEvents();

    addCommands(
        AutonUtils.flipFieldAndResetPose(swerve, trajectory),
        events.scoreConeMid(),
        events.intakeCubeAfterScore(),
        new FollowPathWithEvents(
            driveSubsystem.trajectoryFollowerCommand(trajectory),
            trajectory.getMarkers(),
            events.get()));

    if (doBalance) {
      addCommands(climbCommand.tiltRamp(), climbCommand.balance());
    }
  }
}
