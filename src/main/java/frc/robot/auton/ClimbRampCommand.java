package frc.robot.auton;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.vendor.sensor.ADIS16470;
import frc.robot.constants.Drivetrain;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ClimbRampCommand {
  private DriveSubsystem m_drive;
  private ADIS16470 m_imu;

  public ClimbRampCommand(DriveSubsystem drive, ADIS16470 imu) {
    m_drive = drive;
    m_imu = imu;
  }

  public Command tiltRamp() {
    return new DriveToRampCommand(
            m_drive,
            Drivetrain.kXController,
            Drivetrain.kYController,
            Drivetrain.kThetaController,
            new PathConstraints(2.2, 6.0),
            () -> m_drive.getPose())
        .andThen(
            new InstantCommand(() -> m_drive.setX()), Commands.print("X"), new WaitCommand(0.2));
  }

  public Command balance() {
    return new BalanceCommand(m_drive, m_imu, 0.15);
  }
}
