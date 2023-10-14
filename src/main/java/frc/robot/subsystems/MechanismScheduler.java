package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Arm;
import frc.robot.constants.Intake;
import frc.robot.constants.Telescope;
import frc.robot.constants.Wrist;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.telescope.TelescopeSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class MechanismScheduler {
  public enum PresetType {
    HOME,
    INTAKE,
    VERTICAL_CONE_INTAKE,
    SCORE_MID,
    SCORE_HIGH,
    RELEASE,
    SCORE_FRONT_MID,
    SCORE_FRONT_HIGH,
    INTAKE_REAR,
    HYBRID
  }

  private double m_scheduledWristPos = Wrist.kLeftPosition;
  private boolean m_coneMode = true;
  private PWMSparkMax m_blinkin = new PWMSparkMax(0);

  private ArmSubsystem m_arm;
  private TelescopeSubsystem m_telescope;
  private WristSubsystem m_wrist;
  private IntakeSubsystem m_intake;

  public MechanismScheduler(
      ArmSubsystem arm,
      TelescopeSubsystem telescope,
      WristSubsystem wrist,
      IntakeSubsystem intake) {
    m_arm = arm;
    m_telescope = telescope;
    m_wrist = wrist;
    m_intake = intake;
    m_blinkin.set(0.69);
  }

  public Command schedulePreset(PresetType preset) {
    SequentialCommandGroup commands = new SequentialCommandGroup();

    double arm_pos = m_arm.getPositionRadians();

    if (arm_pos < Arm.kSafePosition) { // Arm below safe position
      // commands.addCommands(m_arm.runToPosition(Arm.kSafePosition));

      switch (preset) {
        case HOME:
          commands.addCommands(
              m_arm.runToPosition(Arm.kSafePosition, Arm.kSafePosition - Arm.kLowPresetOverlap));
          commands.addCommands(
              m_telescope
                  .runToPosition(
                      Telescope.kHomePosition,
                      Telescope.kHomePosition - Telescope.kLowPresetOverlap)
                  .deadlineWith(
                      m_wrist.runToPosition(
                          m_coneMode ? Wrist.kConeIntakePosition : Wrist.kCubeIntakePosition)));
          commands.addCommands(m_arm.runToPosition(Arm.kHomePosition));
          break;
        case VERTICAL_CONE_INTAKE:
          commands.addCommands(
              m_arm.runToPosition(Arm.kSafePosition, Arm.kSafePosition - Arm.kLowPresetOverlap));
          commands.addCommands(
              m_telescope
                  .runToPosition(
                      Telescope.kVerticalConeIntakePosition,
                      Telescope.kVerticalConeIntakePosition - Telescope.kLowPresetOverlap)
                  .deadlineWith(m_wrist.runToPosition(Wrist.kRightPosition)));
          commands.addCommands(m_arm.runToPosition(Arm.kVerticalConeIntakePosition));
          break;
        case INTAKE:
          commands.addCommands(
              m_arm.runToPosition(
                  Arm.kSafePosition,
                  Math.abs(m_wrist.getPosition() - Wrist.kConeIntakePosition)
                              <= Wrist.kPositionTolerance
                          && m_coneMode
                      ? Arm.kSafePosition - Arm.kLowPresetOverlap
                      : Arm.kSafePosition)); // make sure the wrist doesn't have to flip to go to
          // intake
          if (m_coneMode) {
            commands.addCommands(
                m_telescope
                    .runToPosition(
                        Telescope.kConeIntakePosition,
                        Telescope.kConeIntakePosition - Telescope.kLowPresetOverlap)
                    .deadlineWith(m_wrist.runToPosition(Wrist.kConeIntakePosition)));
            commands.addCommands(m_arm.runToPosition(Arm.kConeIntakePosition));
          } else {
            commands.addCommands(
                m_telescope
                    .runToPosition(
                        Telescope.kCubeIntakePosition,
                        Telescope.kCubeIntakePosition - Telescope.kLowPresetOverlap)
                    .deadlineWith(m_wrist.runToPosition(Wrist.kCubeIntakePosition)));
            commands.addCommands(m_arm.runToPosition(Arm.kCubeIntakePosition));
          }
          break;
        case SCORE_MID:
          if (m_coneMode) {
            if (Math.abs(m_arm.getPositionRadians() - Arm.kHomePosition) < Arm.kPositionTolerance) {
              commands.addCommands(
                  new ParallelDeadlineGroup(
                      m_arm.armClearGreater(Arm.kSafePosition),
                      m_arm.runToPosition(Arm.kMidConePosition),
                      m_telescope.runToPosition(Telescope.kHomePosition)));
              commands.addCommands(
                  new InstantCommand(() -> m_wrist.setPosition(m_scheduledWristPos)));
              commands.addCommands(m_arm.telescopeClear());
              commands.addCommands(m_telescope.runToPosition(Telescope.kMidConePosition));
            } else {
              commands.addCommands(m_arm.runToPosition(Arm.kSafePosition));
              commands.addCommands(m_telescope.runToPosition(Telescope.kHomePosition));
              commands.addCommands(
                  new InstantCommand(() -> m_wrist.setPosition(m_scheduledWristPos)));
              commands.addCommands(
                  new ParallelRaceGroup(
                      m_arm.runToPosition(Arm.kMidConePosition), m_arm.telescopeClear()));
              commands.addCommands(m_telescope.runToPosition(Telescope.kMidConePosition));
            }
          } else {
            if (Math.abs(m_arm.getPositionRadians() - Arm.kHomePosition) < Arm.kPositionTolerance) {
              commands.addCommands(
                  new ParallelDeadlineGroup(
                      m_arm.armClearGreater(Arm.kSafePosition),
                      m_arm.runToPosition(Arm.kMidCubePosition),
                      m_telescope.runToPosition(Telescope.kHomePosition)));
              commands.addCommands(
                  new InstantCommand(() -> m_wrist.setPosition(Wrist.kConeIntakePosition)));
              commands.addCommands(m_arm.telescopeClear());
              commands.addCommands(m_telescope.runToPosition(Telescope.kMidCubePosition));
            } else {
              commands.addCommands(m_arm.runToPosition(Arm.kSafePosition));
              commands.addCommands(m_telescope.runToPosition(Telescope.kHomePosition));
              commands.addCommands(
                  new InstantCommand(() -> m_wrist.setPosition(Wrist.kConeIntakePosition)));
              commands.addCommands(
                  new ParallelRaceGroup(
                      m_arm.runToPosition(Arm.kMidCubePosition), m_arm.telescopeClear()));
              commands.addCommands(m_telescope.runToPosition(Telescope.kMidCubePosition));
            }
          }
          break;
        case SCORE_HIGH:
          if (m_coneMode) {
            if (Math.abs(m_arm.getPositionRadians() - Arm.kHomePosition) < Arm.kPositionTolerance) {
              commands.addCommands(
                  new ParallelDeadlineGroup(
                      m_arm.armClearGreater(Arm.kSafePosition),
                      m_arm.runToPosition(Arm.kHighConePosition),
                      m_telescope.runToPosition(Telescope.kHomePosition)));
              commands.addCommands(
                  new InstantCommand(() -> m_wrist.setPosition(m_scheduledWristPos)));
              commands.addCommands(m_arm.telescopeClear());
              commands.addCommands(m_telescope.runToPosition(Telescope.kHighConePosition));
            } else {
              commands.addCommands(m_arm.runToPosition(Arm.kSafePosition));
              commands.addCommands(m_telescope.runToPosition(Telescope.kHomePosition));
              commands.addCommands(
                  new InstantCommand(() -> m_wrist.setPosition(m_scheduledWristPos)));
              commands.addCommands(
                  new ParallelRaceGroup(
                      m_arm.runToPosition(Arm.kHighConePosition), m_arm.telescopeClear()));
              commands.addCommands(m_telescope.runToPosition(Telescope.kHighConePosition));
            }
          } else {
            if (Math.abs(m_arm.getPositionRadians() - Arm.kHomePosition) < Arm.kPositionTolerance) {
              commands.addCommands(
                  new ParallelDeadlineGroup(
                      m_arm.armClearGreater(Arm.kSafePosition),
                      m_arm.runToPosition(Arm.kHighCubePosition),
                      m_telescope.runToPosition(Telescope.kHomePosition)));
              commands.addCommands(
                  new InstantCommand(() -> m_wrist.setPosition(Wrist.kConeIntakePosition)));
              commands.addCommands(m_arm.telescopeClear());
              commands.addCommands(m_telescope.runToPosition(Telescope.kHighCubePosition));
            } else {
              commands.addCommands(m_arm.runToPosition(Arm.kSafePosition));
              commands.addCommands(m_telescope.runToPosition(Telescope.kHomePosition));
              commands.addCommands(
                  new InstantCommand(() -> m_wrist.setPosition(Wrist.kConeIntakePosition)));
              commands.addCommands(
                  new ParallelRaceGroup(
                      m_arm.runToPosition(Arm.kHighCubePosition), m_arm.telescopeClear()));
              commands.addCommands(m_telescope.runToPosition(Telescope.kHighCubePosition));
            }
          }
          break;
        case SCORE_FRONT_MID:
          if (m_coneMode) {
            commands.addCommands(m_arm.runToPosition(Arm.kFrontMidConePosition));
            commands.addCommands(
                new ParallelCommandGroup(
                    m_telescope.runToPosition(Telescope.kFrontMidConePosition),
                    m_wrist.runToPosition(invertFutureWristPosition())));
          } else {
            commands.addCommands(m_arm.runToPosition(Arm.kFrontMidCubePosition));
            commands.addCommands(
                new ParallelCommandGroup(
                    m_telescope.runToPosition(Telescope.kFrontMidCubePosition),
                    m_wrist.runToPosition(Wrist.kCubeIntakePosition)));
          }
          break;
        case SCORE_FRONT_HIGH:
          if (m_coneMode) {
            /*commands.addCommands(m_telescope.runToPosition(Telescope.kHomePosition));
            commands.addCommands(
                new ParallelCommandGroup(
                    m_arm.runToPosition(Arm.kHighConePosition),
                    m_wrist.runToPosition(m_scheduledWristPos)));
            commands.addCommands(m_telescope.runToPosition(Telescope.kHighConePosition));*/
          } else {
            commands.addCommands(m_arm.runToPosition(Arm.kFrontHighCubePosition));
            commands.addCommands(
                new ParallelCommandGroup(
                    m_telescope.runToPosition(Telescope.kFrontHighCubePosition),
                    m_wrist.runToPosition(Wrist.kCubeIntakePosition)));
          }
          break;
        case INTAKE_REAR:
          commands.addCommands(m_arm.runToPosition(Arm.kSafePosition));
          if (m_coneMode) {
            commands.addCommands(m_telescope.runToPosition(Telescope.kHomePosition));
            commands.addCommands(
                new ParallelCommandGroup(
                    m_arm.runToPosition(Arm.kRearConeIntakePosition),
                    m_wrist.runToPosition(Wrist.kCubeIntakePosition)));
            commands.addCommands(m_telescope.runToPosition(Telescope.kConeIntakePosition));
          } else {
            commands.addCommands(m_telescope.runToPosition(Telescope.kHomePosition));
            commands.addCommands(
                new ParallelCommandGroup(
                    m_arm.runToPosition(Arm.kRearCubeIntakePosition),
                    m_wrist.runToPosition(Wrist.kConeIntakePosition)));
            commands.addCommands(m_telescope.runToPosition(Telescope.kCubeIntakePosition));
          }
          break;
        case HYBRID:
          commands.addCommands(
              m_arm.runToPosition(
                  Arm.kSafePosition,
                  Math.abs(m_wrist.getPosition() - Wrist.kConeIntakePosition)
                              <= Wrist.kPositionTolerance
                          && m_coneMode
                      ? Arm.kSafePosition - Arm.kLowPresetOverlap
                      : Arm.kSafePosition)); // make sure the wrist doesn't have to flip to go to
          // intake
          if (m_coneMode) {
            commands.addCommands(
                m_telescope
                    .runToPosition(
                        Telescope.kHybridConePosition,
                        Telescope.kHybridConePosition - Telescope.kLowPresetOverlap)
                    .deadlineWith(m_wrist.runToPosition(Wrist.kConeIntakePosition)));
            commands.addCommands(m_arm.runToPosition(Arm.kHybridConePosition));
            commands.addCommands(m_intake.outtakeCommand(Intake.kConeHybridVoltage, 1.0));
          } else {
            commands.addCommands(
                m_telescope
                    .runToPosition(
                        Telescope.kHybridCubePosition,
                        Telescope.kHybridCubePosition - Telescope.kLowPresetOverlap)
                    .deadlineWith(m_wrist.runToPosition(Wrist.kCubeIntakePosition)));
            commands.addCommands(m_arm.runToPosition(Arm.kHybridCubePosition));
            commands.addCommands(m_intake.outtakeCommand(Intake.kCubeHybridVoltage, 1.0));
          }
          break;
      }
    } else { // Arm above safe position
      // check for release preset first
      if (preset == PresetType.RELEASE) {
        if (m_coneMode) {
          // closer to mid preset?
          if (Math.abs(m_telescope.getPosition() - Telescope.kMidConePosition)
              < Math.abs(m_telescope.getPosition() - Telescope.kHighConePosition)) {
            return new InstantCommand(
                () -> {
                  m_arm.setPosition(Arm.kMidReleasePosition);
                });
          } else { // closer to high preset?
            return new InstantCommand(
                () -> {
                  m_arm.setPosition(Arm.kHighReleasePosition);
                });
          }
        }
      }

      switch (preset) {
        case HOME:
          commands.addCommands(
              new ParallelRaceGroup(
                  m_telescope.runToPosition(Telescope.kHomePosition), m_telescope.wristClear()));
          if (m_arm.getPositionRadians() > 1.57) {
            commands.addCommands(
                new ParallelCommandGroup(
                    m_wrist.runToPosition(
                        m_coneMode ? Wrist.kConeIntakePosition : Wrist.kCubeIntakePosition),
                    m_arm.runToPosition(Arm.kHomePosition)));
          } else {
            commands.addCommands(
                m_wrist.runToPosition(
                    m_coneMode ? Wrist.kConeIntakePosition : Wrist.kCubeIntakePosition),
                m_arm.runToPosition(Arm.kHomePosition));
          }

          break;
        case VERTICAL_CONE_INTAKE:
          commands.addCommands(m_telescope.runToPosition(Telescope.kHomePosition));
          commands.addCommands(
              new ParallelCommandGroup(
                  m_arm.runToPosition(Arm.kSafePosition),
                  m_wrist.runToPosition(Wrist.kRightPosition)));
          commands.addCommands(
              m_telescope.runToPosition(Telescope.kVerticalConeIntakePosition),
              m_arm.runToPosition(Arm.kVerticalConeIntakePosition));
          break;
        case INTAKE:
          commands.addCommands(
              new ParallelRaceGroup(
                  m_telescope.runToPosition(Telescope.kHomePosition), m_telescope.wristClear()));
          if (m_coneMode) {
            if (m_arm.getPositionRadians() > 1.57) {
              commands.addCommands(
                  new ParallelDeadlineGroup(
                      m_arm.armClearLess(0.0),
                      m_arm.runToPosition(Arm.kConeIntakePosition),
                      m_wrist.runToPosition(Wrist.kConeIntakePosition)));
              commands.addCommands(m_telescope.runToPosition(Telescope.kConeIntakePosition));
            } else {
              commands.addCommands(
                  new ParallelCommandGroup(
                      m_arm.runToPosition(Arm.kSafePosition),
                      m_wrist.runToPosition(Wrist.kConeIntakePosition)));
              commands.addCommands(
                  m_telescope.runToPosition(Telescope.kConeIntakePosition),
                  m_arm.runToPosition(Arm.kConeIntakePosition));
            }

          } else {
            if (m_arm.getPositionRadians() > 1.57) {
              commands.addCommands(
                  new ParallelDeadlineGroup(
                      m_arm.armClearLess(0.0),
                      m_arm.runToPosition(Arm.kCubeIntakePosition),
                      m_wrist.runToPosition(Wrist.kCubeIntakePosition)));
              commands.addCommands(m_telescope.runToPosition(Telescope.kCubeIntakePosition));
            } else {
              commands.addCommands(
                  new ParallelCommandGroup(
                      m_arm.runToPosition(Arm.kSafePosition),
                      m_wrist.runToPosition(Wrist.kCubeIntakePosition)));
              commands.addCommands(
                  m_telescope.runToPosition(Telescope.kCubeIntakePosition),
                  m_arm.runToPosition(Arm.kCubeIntakePosition));
            }
          }
          break;
        case SCORE_MID:
          commands.addCommands(m_telescope.runToPosition(Telescope.kHomePosition));
          if (m_coneMode) {
            commands.addCommands(
                new ParallelCommandGroup(
                    m_arm.runToPosition(Arm.kMidConePosition),
                    m_wrist.runToPosition(m_scheduledWristPos)));
            commands.addCommands(m_telescope.runToPosition(Telescope.kMidConePosition));
          } else {
            commands.addCommands(
                new ParallelCommandGroup(
                    m_arm.runToPosition(Arm.kMidCubePosition),
                    m_wrist.runToPosition(Wrist.kConeIntakePosition)));
            commands.addCommands(m_telescope.runToPosition(Telescope.kMidCubePosition));
          }
          break;
        case SCORE_HIGH:
          commands.addCommands(m_telescope.runToPosition(Telescope.kHomePosition));
          if (m_coneMode) {
            commands.addCommands(
                new ParallelCommandGroup(
                    m_arm.runToPosition(Arm.kHighConePosition),
                    m_wrist.runToPosition(m_scheduledWristPos)));
            commands.addCommands(m_telescope.runToPosition(Telescope.kHighConePosition));
          } else {
            commands.addCommands(
                new ParallelCommandGroup(
                    m_arm.runToPosition(Arm.kHighCubePosition),
                    m_wrist.runToPosition(Wrist.kConeIntakePosition)));
            commands.addCommands(m_telescope.runToPosition(Telescope.kHighCubePosition));
          }
          break;
        case SCORE_FRONT_MID:
          commands.addCommands(m_telescope.runToPosition(Telescope.kHomePosition));
          if (m_coneMode) {
            commands.addCommands(
                new ParallelCommandGroup(
                    m_arm.runToPosition(Arm.kFrontMidConePosition),
                    m_wrist.runToPosition(invertFutureWristPosition())));
            commands.addCommands(m_telescope.runToPosition(Telescope.kFrontMidConePosition));
          } else {
            commands.addCommands(
                new ParallelCommandGroup(
                    m_arm.runToPosition(Arm.kFrontMidCubePosition),
                    m_wrist.runToPosition(Wrist.kCubeIntakePosition)));
            commands.addCommands(m_telescope.runToPosition(Telescope.kFrontMidCubePosition));
          }
          break;
        case SCORE_FRONT_HIGH:
          commands.addCommands(m_telescope.runToPosition(Telescope.kHomePosition));
          if (m_coneMode) {
            /*commands.addCommands(
                new ParallelCommandGroup(
                    m_arm.runToPosition(Arm.kHighConePosition),
                    m_wrist.runToPosition(m_scheduledWristPos)));
            commands.addCommands(m_telescope.runToPosition(Telescope.kHighConePosition));*/
          } else {
            commands.addCommands(
                new ParallelCommandGroup(
                    m_arm.runToPosition(Arm.kFrontHighCubePosition),
                    m_wrist.runToPosition(Wrist.kCubeIntakePosition)));
            commands.addCommands(m_telescope.runToPosition(Telescope.kFrontHighCubePosition));
          }
          break;
        case INTAKE_REAR:
          commands.addCommands(m_telescope.runToPosition(Telescope.kHomePosition));
          if (m_coneMode) {
            commands.addCommands(
                new ParallelCommandGroup(
                    m_arm.runToPosition(Arm.kRearConeIntakePosition),
                    m_wrist.runToPosition(Wrist.kCubeIntakePosition)));
            commands.addCommands(m_telescope.runToPosition(Telescope.kConeIntakePosition));
          } else {
            commands.addCommands(
                new ParallelCommandGroup(
                    m_arm.runToPosition(Arm.kRearCubeIntakePosition),
                    m_wrist.runToPosition(Wrist.kConeIntakePosition)));
            commands.addCommands(m_telescope.runToPosition(Telescope.kCubeIntakePosition));
          }
          break;
        case HYBRID:
          commands.addCommands(m_telescope.runToPosition(Telescope.kHomePosition));
          if (m_coneMode) {
            commands.addCommands(
                new ParallelCommandGroup(
                    m_arm.runToPosition(Arm.kSafePosition),
                    m_wrist.runToPosition(Wrist.kConeIntakePosition)));
            commands.addCommands(
                m_telescope.runToPosition(Telescope.kHybridConePosition),
                m_arm.runToPosition(Arm.kHybridConePosition));
            commands.addCommands(m_intake.outtakeCommand(Intake.kConeHybridVoltage, 1.0));
          } else {
            commands.addCommands(
                new ParallelCommandGroup(
                    m_arm.runToPosition(Arm.kSafePosition),
                    m_wrist.runToPosition(Wrist.kCubeIntakePosition)));
            commands.addCommands(
                m_telescope.runToPosition(Telescope.kHybridCubePosition),
                m_arm.runToPosition(Arm.kHybridCubePosition));
            commands.addCommands(m_intake.outtakeCommand(Intake.kCubeHybridVoltage, 1.0));
          }
          break;
      }
    }

    return commands;
  }

  public void setFutureWristPosition(double pos) {
    m_scheduledWristPos = pos;
  }

  /**
   * Used for inverting the left/right future wrist position for when mechanism scheduling front
   * cone scoring positions
   *
   * @param _wristAngle
   * @return
   * @throws Exception
   */
  private double invertFutureWristPosition() {
    if (m_scheduledWristPos == Wrist.kLeftPosition) {
      return Wrist.kRightPosition;
    } else if (m_scheduledWristPos == Wrist.kRightPosition) {
      return Wrist.kLeftPosition;
    } else {
      return m_scheduledWristPos;
    }
  }

  public boolean getConeMode() {
    return m_coneMode;
  }

  public void setConeMode() {
    m_coneMode = true;
    m_blinkin.set(0.69);
  }

  public void setCubeMode() {
    m_coneMode = false;
    m_blinkin.set(0.91);
  }

  public boolean isInHybridZone() {
    if (m_arm.getPositionRadians() < 0.0) {
      if (m_coneMode) {
        return Math.abs(m_arm.getPositionRadians() - Arm.kHybridConePosition)
            < Math.abs(m_arm.getPositionRadians() - Arm.kConeIntakePosition);
      } else { // cube mode
        return Math.abs(m_arm.getPositionRadians() - Arm.kHybridCubePosition)
            < Math.abs(m_arm.getPositionRadians() - Arm.kCubeIntakePosition);
      }
    } else {
      return false;
    }
  }
}
