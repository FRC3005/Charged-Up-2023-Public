package frc.robot.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.vendor.sensor.ADIS16470;
import frc.robot.auton.locations.CenterAutonLocation;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotstate.RobotState;

public class CenterBetterCubeAuto extends AutonCommandBase {

  private static final double pathMaxVel = 2.5;
  private static final double pathMaxAccel = 4.0;

  private static PathPlannerTrajectory firstPath =
      PathPlanner.loadPath("CenterGrabOneFirst", pathMaxVel, pathMaxAccel);
  private static PathPlannerTrajectory secondPath =
      PathPlanner.loadPath("CenterGrabOneSecond", 1.4, pathMaxAccel);
  // Change the below path to just "LoadingThird" to grab the 3rd cube
  private static PathPlannerTrajectory thirdPath =
      PathPlanner.loadPath("CenterGrabOneThird", pathMaxVel, pathMaxAccel);
  DriveSubsystem driveSubsystem;
  private static PathPlannerTrajectory fourthPath =
      PathPlanner.loadPath("CenterGrabOneFourth", 1.5, pathMaxAccel);

  public CenterBetterCubeAuto(DriveSubsystem swerve, ADIS16470 imu, RobotState rs) {
    driveSubsystem = swerve;

    var events = new AutonEvents();
    ClimbRampCommand climbCommand = new ClimbRampCommand(swerve, imu);
    FollowPathWithEvents firstCubePath =
        new FollowPathWithEvents(
            driveSubsystem.trajectoryFollowerCommand(firstPath),
            firstPath.getMarkers(),
            events.get());

    FollowPathWithEvents secondCubePath =
        new FollowPathWithEvents(
            driveSubsystem.trajectoryFollowerCommand(secondPath),
            secondPath.getMarkers(),
            events.get());

    FollowPathWithEvents thirdCubePath =
        new FollowPathWithEvents(
            driveSubsystem.trajectoryFollowerCommand(thirdPath),
            thirdPath.getMarkers(),
            events.get());

    FollowPathWithEvents fourthCubePath =
        new FollowPathWithEvents(
            driveSubsystem.trajectoryFollowerCommand(fourthPath),
            fourthPath.getMarkers(),
            events.get());

    addCommands(
        AutonUtils.flipFieldAndResetPose(swerve, rs, firstPath),
        events.scoreConeHighStart(),
        // events.armSafeCubeIntake(),
        firstCubePath, // This path starts the intake
        secondCubePath,
        thirdCubePath,
        AutonUtils.flipFieldAndResetPose(swerve, rs, fourthPath),
        fourthCubePath,
        events.scoreCubeHigh(),
        new InstantCommand(() -> swerve.resetOdometry(CenterAutonLocation.get()), swerve),
        climbCommand
            .tiltRamp()
            .alongWith(new WaitCommand(0.6).andThen(events.intakeHomeAfterScore())),
        climbCommand.balance());
  }
}
