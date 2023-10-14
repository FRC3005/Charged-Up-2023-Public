package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.vendor.sensor.ADIS16470;
import frc.robot.auton.locations.CenterAutonLocation;
import frc.robot.subsystems.drive.DriveSubsystem;

public class CenterScoreAndBalance extends AutonCommandBase {

  DriveSubsystem driveSubsystem;

  public CenterScoreAndBalance(DriveSubsystem swerve, ADIS16470 imu) {
    driveSubsystem = swerve;
    ClimbRampCommand climbCommand = new ClimbRampCommand(swerve, imu);

    var events = new AutonEvents();

    addCommands(
        new InstantCommand(() -> swerve.resetOdometry(CenterAutonLocation.get()), swerve)
            .withName("Reset Odometry"),
        events.scoreConeHighStart(),
        events.telescopeHome(),
        climbCommand
            .tiltRamp()
            .alongWith(new WaitCommand(1.0).andThen(events.intakeHomeAfterScore())),
        climbCommand.balance());
  }
}
