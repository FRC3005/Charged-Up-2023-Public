package frc.robot.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.vendor.sensor.ADIS16470;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.List;

public class CenterTwoBalance extends AutonCommandBase {
  private static final double kOverRampVelocity = 1.8;
  private static final double kOverRampAcceleration = 6.5;

  private static List<PathPlannerTrajectory> pathGroup =
      PathPlanner.loadPathGroup(
          "CenterDouble",
          new PathConstraints(kOverRampVelocity, kOverRampAcceleration),
          new PathConstraints(kOverRampVelocity, kOverRampAcceleration),
          new PathConstraints(2.0, kOverRampAcceleration));
  DriveSubsystem driveSubsystem;

  public CenterTwoBalance(DriveSubsystem swerve, ADIS16470 imu) {
    driveSubsystem = swerve;
    ClimbRampCommand climbCommand = new ClimbRampCommand(swerve, imu);

    var events = new AutonEvents();
    FollowPathWithEvents path1 =
        new FollowPathWithEvents(
            driveSubsystem.trajectoryFollowerCommand(pathGroup.get(0)),
            pathGroup.get(0).getMarkers(),
            events.get());

    FollowPathWithEvents path2 =
        new FollowPathWithEvents(
            driveSubsystem.trajectoryFollowerCommand(pathGroup.get(1)),
            pathGroup.get(1).getMarkers(),
            events.get());

    FollowPathWithEvents path3 =
        new FollowPathWithEvents(
            driveSubsystem.trajectoryFollowerCommand(pathGroup.get(2)),
            pathGroup.get(2).getMarkers(),
            events.get());

    addCommands(
        AutonUtils.flipFieldAndResetPose(swerve, pathGroup.get(0)),
        events.scoreConeHighStart(),
        path1.alongWith(new WaitCommand(1.0).andThen(events.intakeCubeAfterScore())),
        path2.alongWith(events.intakeOff()),
        events.scoreCubeHigh(),
        path3.alongWith(new WaitCommand(0.75).andThen(events.intakeHome())),
        climbCommand.balance());
  }
}
