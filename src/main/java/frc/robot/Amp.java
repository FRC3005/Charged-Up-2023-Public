// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.telemetry.TelemetryRunner;
import frc.lib.util.JoystickUtil;
import frc.lib.util.SendableJVM;
import frc.lib.vendor.motorcontroller.SparkMax;
import frc.lib.vendor.sensor.ADIS16470;
import frc.lib.vendor.sensor.ADIS16470.ADIS16470CalibrationTime;
import frc.robot.auton.AutonChooser;
import frc.robot.auton.AutonEvents;
import frc.robot.auton.BumpThreePiece;
import frc.robot.auton.CenterBetterCubeAuto;
import frc.robot.auton.CenterScoreAndBalance;
import frc.robot.auton.CenterScoreGrabCubeAndBalance;
import frc.robot.auton.CenterTwoBalance;
import frc.robot.auton.LoadingStationAuton;
import frc.robot.auton.ScoreAndDriveForward;
import frc.robot.constants.*;
import frc.robot.subsystems.MechanismScheduler;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.robotstate.RobotState;
import frc.robot.subsystems.telescope.TelescopeSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import org.tinylog.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class Amp {
  private final ADIS16470 m_gyro =
      new ADIS16470(
          RobotConstants.kCompetitionMode
              ? ADIS16470CalibrationTime._8s
              : ADIS16470CalibrationTime._1s);
  private final DriveSubsystem m_drive = new DriveSubsystem(m_gyro);

  private final SimMech m_simMech = new SimMech();

  private final ArmSubsystem m_arm = new ArmSubsystem(m_simMech);
  private final TelescopeSubsystem m_telescope = new TelescopeSubsystem(m_simMech);
  private final WristSubsystem m_wrist = new WristSubsystem(m_simMech);
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final MechanismScheduler m_mechanismScheduler =
      new MechanismScheduler(m_arm, m_telescope, m_wrist, m_intake);

  public final SimMech m_simBase = new SimMech();

  XboxController m_driveController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  private Timer m_armResetTimer = new Timer();

  private final RobotState m_robotState = RobotState.getInstance(m_drive, m_gyro);

  // private boolean m_coneMode = true;

  private String m_selectedAutonName = "";

  // another option; we can specify getters Oblog calls on the field
  // @Log(name="Selected Auton", methodName="getName")
  // private Command m_selectedAuton = Commands.runOnce(() -> {});

  /* List Autons here */
  {
    AutonEvents.setup(m_arm, m_telescope, m_wrist, m_intake);
    new ScoreAndDriveForward(m_drive);
    new LoadingStationAuton(m_drive, m_gyro, m_robotState, true);
    new LoadingStationAuton(m_drive, m_gyro, m_robotState, false);
    // new LoadingScoreOneBalance(m_drive, m_gyro, false);
    // new LoadingScoreOneBalance(m_drive, m_gyro, true);
    new CenterScoreAndBalance(m_drive, m_gyro);
    new CenterScoreGrabCubeAndBalance(m_drive, m_gyro, false);
    new CenterBetterCubeAuto(m_drive, m_gyro, m_robotState);
    // new CenterBalanceOnly(m_drive, m_gyro);
    // new CenterScoreGrabCubeYeetAndBalance(m_drive, m_gyro);
    new CenterTwoBalance(m_drive, m_gyro);
    // new CenterScoreGrabOneBalance(m_drive, m_gyro);
    // new BumpThreePiece(m_drive, m_gyro, m_robotState, true);
    new BumpThreePiece(m_drive, m_gyro, m_robotState, false);
    // new BumpScoreOneBalance(m_drive, m_gyro, true);
    // new BumpScoreOneGrabCubeBalance(m_drive, m_gyro, false);
    // new BumpScoreOneGrabCubeBalance(m_drive, m_gyro, true);
    // new SwerveRotateTuning(m_drive);
    // new SwerveDriveTuning(m_drive);
    // new SwerveDriveBackTuning(m_drive);
    // new SwerveCombinedTuning(m_drive);
    // new LoadingStationTwo(m_drive);
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public Amp() {
    // ALL Spark Maxes should have already been initialized by this point
    SparkMax.burnFlashInSync();

    // Configure the button bindings
    configureButtonBindings();
    configureTestModeBindings();

    SmartDashboard.putData("JVM", new SendableJVM());
    SmartDashboard.putNumber("Slow Mode Drive Scalar", 0.6);
    SmartDashboard.putNumber("Slow Mode Turn Scalar", 0.6);

    TelemetryRunner.getDefault().bind(m_drive);
    TelemetryRunner.getDefault().bind(m_arm);
    TelemetryRunner.getDefault().bind(m_telescope);
    TelemetryRunner.getDefault().bind(m_wrist);
    TelemetryRunner.getDefault().bind(m_robotState);
    TelemetryRunner.getDefault().bind(m_intake);

    if (RobotConstants.kCompetitionMode) {
      Logger.tag("Amp").info("=== Starting Amp in Competition Mode ===");
    } else {
      Logger.tag("Amp").warn("=== Starting Amp in Non-Competition Mode ===");
    }
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

    m_arm.setDefaultCommand(
        new RunCommand(
            () ->
                m_arm.jog(
                    JoystickUtil.squareAxis(
                            m_driveController.getLeftTriggerAxis()
                                - m_driveController.getRightTriggerAxis())
                        * -0.05),
            m_arm));

    new JoystickButton(m_driveController, XboxController.Button.kRightStick.value)
        .onFalse(
            Commands.runOnce(
                () -> {
                  m_drive.zeroHeading();
                }));

    new JoystickButton(m_driveController, XboxController.Button.kRightBumper.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_intake.setVoltage(
                      m_mechanismScheduler.getConeMode()
                          ? Intake.kConeInVoltage
                          : Intake.kCubeInVoltage);
                },
                m_intake))
        .onFalse(
            new InstantCommand(
                () -> {
                  if (m_mechanismScheduler.getConeMode()) {
                    m_intake.setVoltage(3.0);
                  } else {

                    m_intake.setVoltage(0.0);
                  }
                },
                m_intake));
    new JoystickButton(m_driveController, XboxController.Button.kLeftBumper.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_intake.setVoltage(
                      m_mechanismScheduler.getConeMode()
                          ? (m_mechanismScheduler.isInHybridZone()
                              ? Intake.kConeHybridVoltage
                              : Intake.kConeOutVoltage)
                          : (m_mechanismScheduler.isInHybridZone()
                              ? Intake.kCubeHybridVoltage
                              : Intake.kCubeOutVoltage));
                },
                m_intake))
        .onFalse(
            new InstantCommand(
                () -> {
                  m_intake.setVoltage(0.0);
                },
                m_intake));

    new JoystickButton(m_driveController, XboxController.Button.kA.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_mechanismScheduler
                      .schedulePreset(MechanismScheduler.PresetType.RELEASE)
                      .schedule();
                }));

    new Trigger(() -> m_driveController.getPOV() == 0)
        .onTrue(
            new RunCommand(
                () -> {
                  // ~1 in/second
                  m_telescope.jog(-0.0254 / 10.0);
                },
                m_telescope))
        .onFalse(new InstantCommand(() -> m_telescope.jog(0), m_telescope));

    new Trigger(() -> m_driveController.getPOV() == 180)
        .onTrue(
            new RunCommand(
                () -> {
                  // ~1 in/second
                  m_telescope.jog(0.0254 / 10.0);
                },
                m_telescope))
        .onFalse(new InstantCommand(() -> m_telescope.jog(0), m_telescope));

    new JoystickButton(m_driveController, XboxController.Button.kY.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_mechanismScheduler
                      .schedulePreset(MechanismScheduler.PresetType.HYBRID)
                      .schedule();
                }));

    new JoystickButton(m_driveController, XboxController.Button.kX.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_mechanismScheduler
                      .schedulePreset(MechanismScheduler.PresetType.INTAKE)
                      .schedule();
                }));

    new JoystickButton(m_driveController, XboxController.Button.kB.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_mechanismScheduler
                      .schedulePreset(MechanismScheduler.PresetType.HOME)
                      .schedule();
                }));

    /*************************************************
     * Combined controls
     *************************************************/

    /*************************************************
     * Operator controls
     *************************************************/
    new JoystickButton(m_operatorController, XboxController.Button.kStart.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_mechanismScheduler.setConeMode();
                  SmartDashboard.putString("Mode", "Cone");
                }));
    new JoystickButton(m_operatorController, XboxController.Button.kBack.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_mechanismScheduler.setCubeMode();
                  SmartDashboard.putString("Mode", "Ball");
                }));

    // arm -----------------------------------------------------------------------
    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_mechanismScheduler
                      .schedulePreset(MechanismScheduler.PresetType.HOME)
                      .schedule();
                }));
    new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_mechanismScheduler
                      .schedulePreset(MechanismScheduler.PresetType.VERTICAL_CONE_INTAKE)
                      .schedule();
                  m_mechanismScheduler.setFutureWristPosition(Wrist.kLeftPosition);
                }));
    new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_mechanismScheduler
                      .schedulePreset(MechanismScheduler.PresetType.INTAKE_REAR)
                      .schedule();
                }));
    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_mechanismScheduler
                      .schedulePreset(MechanismScheduler.PresetType.INTAKE)
                      .schedule();
                }));
    new JoystickButton(m_operatorController, XboxController.Button.kX.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_mechanismScheduler
                      .schedulePreset(MechanismScheduler.PresetType.SCORE_MID)
                      .schedule();
                }));
    new JoystickButton(m_operatorController, XboxController.Button.kY.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_mechanismScheduler
                      .schedulePreset(MechanismScheduler.PresetType.SCORE_HIGH)
                      .schedule();
                }));
    new Trigger(() -> m_operatorController.getRightTriggerAxis() > 0.5)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_mechanismScheduler
                      .schedulePreset(MechanismScheduler.PresetType.SCORE_FRONT_HIGH)
                      .schedule();
                }));
    /*new Trigger(() -> m_operatorController.getRightTriggerAxis() > 0.5)
    .onTrue(
        new InstantCommand(
            () -> {
              m_mechanismScheduler
                  .schedulePreset(MechanismScheduler.PresetType.SCORE_FRONT_MID)
                  .schedule();
            }));*/

    // wrist -----------------------------------------------------------------------
    new Trigger(() -> m_operatorController.getPOV() == 0)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_wrist.setPosition(Wrist.kCubeIntakePosition);
                },
                m_wrist));
    new Trigger(() -> m_operatorController.getPOV() == 90)
        .onTrue(
            new InstantCommand(
                () -> {
                  if (m_arm.getPositionRadians() > Arm.kSafePosition
                      && m_arm.getPositionRadians() < 3.3) {
                    m_wrist.setPosition(Wrist.kRightPosition);
                    m_mechanismScheduler.setFutureWristPosition(Wrist.kRightPosition);
                  } else {
                    m_mechanismScheduler.setFutureWristPosition(Wrist.kRightPosition);
                  }
                },
                m_wrist));
    new Trigger(() -> m_operatorController.getPOV() == 180)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_wrist.setPosition(Wrist.kConeIntakePosition);
                },
                m_wrist));
    new Trigger(() -> m_operatorController.getPOV() == 270)
        .onTrue(
            new InstantCommand(
                () -> {
                  if (m_arm.getPositionRadians() > Arm.kSafePosition
                      && m_arm.getPositionRadians() < 3.3) {
                    m_wrist.setPosition(Wrist.kLeftPosition);
                    m_mechanismScheduler.setFutureWristPosition(Wrist.kLeftPosition);
                  } else {
                    m_mechanismScheduler.setFutureWristPosition(Wrist.kLeftPosition);
                  }
                },
                m_wrist));
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
    m_arm.testModePeriodic();
    m_telescope.testModePeriodic();
    m_wrist.testModePeriodic();
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
    m_robotState.enableVision();
    m_drive.calibrateGyro();
    m_drive.stop();
  }

  public void teleopInit() {
    m_robotState.enableVision();
  }

  public void disabledInit() {
    m_robotState.enableVision();
    m_armResetTimer.restart();
  }

  private final double kArmResetFrequency = 2.0;

  public void disabledPeriodic() {
    if (m_armResetTimer.hasElapsed(kArmResetFrequency)) {
      m_arm.resetEncoder();
      m_armResetTimer.restart();
    }
  }
}
