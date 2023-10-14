package frc.robot.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.vendor.sensor.ADIS16470;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotstate.RobotState;

public class BumpThreePiece extends AutonCommandBase {

  private static PathPlannerTrajectory trajectory =
      AutonUtils.loadTrajectory("BumpFirst", 5.0, 3.5);
  private static PathPlannerTrajectory trajectory2 =
      AutonUtils.loadTrajectory("BumpSecond", 5.0, 3.5);
  DriveSubsystem driveSubsystem;

  private static PathPlannerTrajectory balanceBackTrajectory =
      AutonUtils.loadTrajectory("BumpToBalanceFromCube", 2.0, 4.5);

  public BumpThreePiece(DriveSubsystem swerve, ADIS16470 imu, RobotState rs, boolean doBalance) {
    super(doBalance ? "BumpTwoBalance" : "BumpThreePiece");
    driveSubsystem = swerve;

    var events = new AutonEvents();
    ClimbRampCommand climbCommand = new ClimbRampCommand(swerve, imu);

    FollowPathWithEvents balanceFrontPath =
        new FollowPathWithEvents(
            driveSubsystem.trajectoryFollowerCommand(balanceBackTrajectory, rs::getPoseEstimate),
            balanceBackTrajectory.getMarkers(),
            events.get());

    addCommands(
        AutonUtils.flipFieldAndResetPose(swerve, rs, trajectory),
        events.scoreConeHighStart(),
        new FollowPathWithEvents(
            driveSubsystem.trajectoryFollowerCommand(trajectory, rs::getPoseEstimate),
            trajectory.getMarkers(),
            events.get()),
        events.postScoreCube());

    if (doBalance) {
      addCommands(
          new InstantCommand(() -> rs.disableVision()), balanceFrontPath, climbCommand.balance());
    } else {
      addCommands(
          new FollowPathWithEvents(
              driveSubsystem.trajectoryFollowerCommand(trajectory2, rs::getPoseEstimate),
              trajectory2.getMarkers(),
              events.get()),
          events.postScoreCube());
    }
  }
}
