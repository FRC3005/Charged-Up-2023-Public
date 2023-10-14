package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.vendor.sensor.ADIS16470;
import frc.robot.auton.locations.CenterAutonLocation;
import frc.robot.subsystems.drive.DriveSubsystem;

public class CenterBalanceOnly extends AutonCommandBase {

  DriveSubsystem driveSubsystem;

  public CenterBalanceOnly(DriveSubsystem swerve, ADIS16470 imu) {
    driveSubsystem = swerve;
    ClimbRampCommand climbCommand = new ClimbRampCommand(swerve, imu);

    addCommands(
        new InstantCommand(() -> swerve.resetOdometry(CenterAutonLocation.get()), swerve)
            .withName("Reset Odometry"),
        climbCommand.tiltRamp(),
        climbCommand.balance());
  }
}
