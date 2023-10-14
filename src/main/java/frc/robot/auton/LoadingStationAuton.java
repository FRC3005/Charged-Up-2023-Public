package frc.robot.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.vendor.sensor.ADIS16470;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotstate.RobotState;

public class LoadingStationAuton extends AutonCommandBase {

  private static final double pathMaxVel = 4.5;
  private static final double pathMaxAccel = 3.6;

  private static PathPlannerTrajectory firstPath =
      PathPlanner.loadPath("LoadingFirst2", pathMaxVel, pathMaxAccel);
  private static PathPlannerTrajectory secondPath =
      PathPlanner.loadPath("LoadingSecond", pathMaxVel, pathMaxAccel);
  // Change the below path to just "LoadingThird" to grab the 3rd cube
  private static PathPlannerTrajectory thirdPath =
      PathPlanner.loadPath("LoadingThirdDrive", pathMaxVel, pathMaxAccel);
  DriveSubsystem driveSubsystem;

  private static PathPlannerTrajectory balanceBackTrajectory =
      AutonUtils.loadTrajectory("LoadingToBalanceFromCube", 2.0, 4.5);

  public LoadingStationAuton(
      DriveSubsystem swerve, ADIS16470 imu, RobotState rs, boolean doBalance) {
    super(doBalance ? "LoadingThreePieceBalance" : "LoadingFourPiece");
    driveSubsystem = swerve;

    var events = new AutonEvents();
    ClimbRampCommand climbCommand = new ClimbRampCommand(swerve, imu);
    FollowPathWithEvents firstCubePath =
        new FollowPathWithEvents(
            driveSubsystem.trajectoryFollowerCommand(firstPath, rs::getPoseEstimate),
            firstPath.getMarkers(),
            events.get());

    FollowPathWithEvents secondCubePath =
        new FollowPathWithEvents(
            driveSubsystem.trajectoryFollowerCommand(secondPath, rs::getPoseEstimate),
            secondPath.getMarkers(),
            events.get());

    FollowPathWithEvents thirdCubePath =
        new FollowPathWithEvents(
            driveSubsystem.trajectoryFollowerCommand(thirdPath, rs::getPoseEstimate),
            thirdPath.getMarkers(),
            events.get());

    FollowPathWithEvents balanceFrontPath =
        new FollowPathWithEvents(
            driveSubsystem.trajectoryFollowerCommand(balanceBackTrajectory, rs::getPoseEstimate),
            balanceBackTrajectory.getMarkers(),
            events.get());

    addCommands(
        AutonUtils.flipFieldAndResetPose(swerve, rs, firstPath),
        events.scoreConeHighStart(),
        // events.armSafeCubeIntake(),
        firstCubePath, // This path starts the intake
        events.postScoreCube(),
        secondCubePath,
        events.postScoreCube());

    if (doBalance) {
      addCommands(
          new InstantCommand(() -> rs.disableVision()), balanceFrontPath, climbCommand.balance());
    } else {
      addCommands(
          thirdCubePath, new InstantCommand(() -> swerve.drive(0.0, 0.0, 0.0, false), swerve));
    }
  }
}
