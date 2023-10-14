// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.telemetry.TelemetryRunner;
import frc.lib.util.JoystickUtil;
import frc.lib.util.SendableJVM;
import frc.lib.vendor.motorcontroller.SparkMax;
import frc.lib.vendor.sensor.ADIS16470;
import frc.lib.vendor.sensor.ADIS16470.ADIS16470CalibrationTime;
import frc.robot.auton.AutonChooser;
import frc.robot.auton.SwerveCombinedTuning;
import frc.robot.auton.SwerveDriveBackTuning;
import frc.robot.auton.SwerveDriveTuning;
import frc.robot.auton.SwerveRotateTuning;
import frc.robot.constants.*;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotstate.RobotState;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class SwerveBot {
  private final ADIS16470 m_gyro =
      new ADIS16470(
          RobotConstants.kCompetitionMode
              ? ADIS16470CalibrationTime._8s
              : ADIS16470CalibrationTime._1s);
  private final DriveSubsystem m_drive = new DriveSubsystem(m_gyro);

  XboxController m_driveController = new XboxController(OIConstants.kDriverControllerPort);

  private final RobotState m_robotState = RobotState.getInstance(m_drive, m_gyro);

  private String m_selectedAutonName = "";

  /* List Autons here */
  {
    new SwerveRotateTuning(m_drive);
    new SwerveDriveTuning(m_drive);
    new SwerveDriveBackTuning(m_drive);
    new SwerveCombinedTuning(m_drive);
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public SwerveBot() {
    // ALL Spark Maxes should have already been initialized by this point
    SparkMax.burnFlashInSync();

    // Configure the button bindings
    configureButtonBindings();
    configureTestModeBindings();

    SmartDashboard.putData("JVM", new SendableJVM());
    SmartDashboard.putNumber("Slow Mode Drive Scalar", 0.6);
    SmartDashboard.putNumber("Slow Mode Turn Scalar", 0.6);

    TelemetryRunner.getDefault().bind(m_drive);
  }

  private void configureButtonBindings() {
    /*************************************************
     * Driver controls
     *************************************************/

    m_drive.setDefaultCommand(
        new RunCommand(
                () -> {
                  var fastMode = m_driveController.getLeftStickButton();
                  double driveScalar = 0.6;
                  double turnScalar = 0.6;
                  if (fastMode) {
                    driveScalar = 1.0;
                    turnScalar = 1.0;
                  }
                  var leftY =
                      JoystickUtil.squareAxis(
                          -m_driveController.getLeftY(), OIConstants.kDriveDeadband, driveScalar);
                  var leftX =
                      JoystickUtil.squareAxis(
                          -m_driveController.getLeftX(), OIConstants.kDriveDeadband, driveScalar);
                  var rightX =
                      JoystickUtil.squareAxis(
                          -m_driveController.getRightX(), OIConstants.kDriveDeadband, turnScalar);
                  // Set robot oriented in slow mode
                  m_drive.drive(leftY, leftX, rightX, true);
                },
                m_drive)
            .withName("Default Drive"));

    new JoystickButton(m_driveController, XboxController.Button.kRightStick.value)
        .onFalse(
            Commands.runOnce(
                () -> {
                  m_drive.zeroHeading();
                }));

    /*************************************************
     * Combined controls
     *************************************************/

    /*************************************************
     * Operator controls
     *************************************************/

  }

  /**
   * Create mappings for use in test mode. This function runs once when this object is created.
   *
   * @return
   */
  private void configureTestModeBindings() {}

  /** This function is called each time testmode is started */
  public void testModeInit() {}

  /**
   * This function is called each loop of testmode. The scheduler is disabled in test mode, so this
   * function should run any necessary loops directly.
   */
  public void testModePeriodic() {
    m_drive.testPeriodic();
    m_robotState.testModePeriodic();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command auton = AutonChooser.getAuton();

    org.tinylog.Logger.tag("Auton").info("Auton selected: {}", auton.getName());

    m_selectedAutonName = auton.getName();

    return auton;
  }

  /** Run a function at the start of auton. */
  public void autonInit() {
    m_drive.calibrateGyro();
    m_drive.stop();
  }

  public void teleopInit() {}

  public void disabledInit() {}

  public void disabledPeriodic() {}
}
