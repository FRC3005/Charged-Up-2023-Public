package frc.robot.auton;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.Arm;
import frc.robot.constants.Intake;
import frc.robot.constants.Telescope;
import frc.robot.constants.Wrist;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.telescope.TelescopeSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import java.util.HashMap;
import org.tinylog.Logger;

public class AutonEvents {
  private static ArmSubsystem s_arm = null;
  private static TelescopeSubsystem s_telescope;
  private static WristSubsystem s_wrist;
  private static IntakeSubsystem s_intake;

  public static void setup(
      ArmSubsystem arm,
      TelescopeSubsystem telescope,
      WristSubsystem wrist,
      IntakeSubsystem intake) {
    s_arm = arm;
    s_telescope = telescope;
    s_wrist = wrist;
    s_intake = intake;
  }

  public AutonEvents() {
    if (s_arm == null) {
      Logger.tag("AutonEvents").error("Need to initialize events first");
    }
  }

  public Command intakeOnCube() {
    return new InstantCommand(() -> s_intake.setVoltage(Intake.kCubeInVoltageAuton));
  }

  public Command intakeOff() {
    return new InstantCommand(() -> s_intake.setVoltage(0.0));
  }

  /*
   * Put the arm in the safe position, put the wirst in cube intake
   * End the command after arm is at 0
   */
  public Command armSafeCubeIntake() {
    return new ParallelCommandGroup(
        new ParallelRaceGroup(
            s_arm.runToPosition(Arm.kSafePosition),
            s_arm.armClearLess(Units.degreesToRadians(120.0))),
        s_wrist.runToPosition(Wrist.kCubeIntakePosition));
  }

  public Command intakeCubeAfterScore() {
    return new SequentialCommandGroup(
            new ParallelCommandGroup(
                s_arm.runToPosition(Arm.kSafePosition),
                s_wrist.runToPosition(Wrist.kCubeIntakePosition)),
            s_telescope.runToPosition(Telescope.kCubeIntakePosition),
            s_arm.runToPosition(Arm.kCubeIntakePosition),
            s_intake.setVoltageCommand(Intake.kCubeInVoltageAuton))
        .withName("Auton-intakeCubeAfterScore");
  }

  public Command intakeConeAfterScore() {
    return new SequentialCommandGroup(
            new ParallelCommandGroup(
                s_wrist.runToPosition(Wrist.kConeIntakePosition),
                s_telescope.runToPosition(Telescope.kConeIntakePosition)),
            s_intake.setVoltageCommand(Intake.kConeInVoltage),
            s_arm.runToPosition(Arm.kConeIntakePosition))
        .withName("Auton-intakeConeAfterScore");
  }

  public Command intakeCube() {
    return new SequentialCommandGroup(
            s_arm.runToPosition(Arm.kSafePosition),
            new ParallelCommandGroup(
                s_wrist.runToPosition(Wrist.kCubeIntakePosition),
                s_telescope.runToPosition(Telescope.kCubeIntakePosition)),
            s_intake.setVoltageCommand(Intake.kCubeInVoltageAuton),
            s_arm.runToPosition(Arm.kCubeIntakePosition))
        .withName("Auton-intakeCube");
  }

  public Command intakeCone() {
    return new SequentialCommandGroup(
            s_arm.runToPosition(Arm.kSafePosition),
            new ParallelCommandGroup(
                s_wrist.runToPosition(Wrist.kConeIntakePosition),
                s_telescope.runToPosition(Telescope.kConeIntakePosition)),
            s_intake.setVoltageCommand(Intake.kConeInVoltage),
            s_arm.runToPosition(Arm.kConeIntakePosition))
        .withName("Auton-intakeCone");
  }

  public Command intakeSafeCube() {
    return new SequentialCommandGroup(
            s_intake.setVoltageCommand(0.0),
            s_arm.runToPosition(Arm.kSafePosition),
            new ParallelCommandGroup(
                s_wrist.runToPosition(Wrist.kCubeIntakePosition),
                s_telescope.runToPosition(Telescope.kHomePosition)))
        .withName("Auton-intakeSafeCube");
  }

  public Command intakeHomeCube() {
    return new SequentialCommandGroup(
            s_intake.setVoltageCommand(0.0),
            s_arm.runToPosition(Arm.kSafePosition),
            new ParallelCommandGroup(
                s_wrist.runToPosition(Wrist.kCubeIntakePosition),
                s_telescope.runToPosition(Telescope.kHomePosition)),
            s_arm.runToPosition(Arm.kHomePosition))
        .withName("Auton-intakeHomeCube");
  }

  public Command intakeHomeAfterScore() {
    return new SequentialCommandGroup(
            s_intake.setVoltageCommand(0.0),
            new ParallelCommandGroup(
                s_wrist.runToPosition(Wrist.kConeIntakePosition),
                s_telescope.runToPosition(Telescope.kHomePosition)),
            s_arm.runToPosition(Arm.kHomePosition))
        .withName("Auton-intakeHomeAfterScore");
  }

  public Command intakeHome() {
    return new SequentialCommandGroup(
            s_intake.setVoltageCommand(0.0),
            s_arm.runToPosition(Arm.kSafePosition),
            new ParallelCommandGroup(
                s_wrist.runToPosition(Wrist.kConeIntakePosition),
                s_telescope.runToPosition(Telescope.kHomePosition)),
            s_arm.runToPosition(Arm.kHomePosition))
        .withName("Auton-intakeHome");
  }

  public Command scoreConeMid() {
    return new SequentialCommandGroup(
            s_intake.setVoltageCommand(2.0),
            new ParallelCommandGroup(
                s_arm.runToPosition(Arm.kMidReleasePosition + 0.1),
                new WaitCommand(0.65).andThen(s_wrist.runToPosition(Wrist.kRightPosition))),
            s_intake.setVoltageCommand(-2.0))
        .withName("Auton-scoreConeMid");
  }

  public Command scoreConeHigh() {
    return new SequentialCommandGroup(
            s_intake.setVoltageCommand(2.0),
            new ParallelRaceGroup(
                s_arm.runToPosition(Arm.kHighConePosition), s_arm.telescopeClear()),
            s_telescope
                .runToPosition(Telescope.kHighConePosition - 0.03)
                .alongWith(s_wrist.runToPosition(Wrist.kRightPosition)),
            new ParallelCommandGroup(
                s_arm.runToPosition(Arm.kHighReleasePosition + 0.1),
                s_intake.setVoltageCommand(-2.0)),
            s_telescope.runToPosition(Telescope.kHomePosition),
            s_intake.setVoltageCommand(0))
        .withName("Auton-scoreConeHigh");
  }

  public Command scoreConeHighStart() {
    return new SequentialCommandGroup(
            s_intake.setVoltageCommand(2.0),
            new ParallelCommandGroup(
                s_arm.runToPosition(Arm.kHighReleasePosition + 0.0),
                s_arm.scheduleWhenArmGreater(
                    s_telescope
                        .runToPosition(Telescope.kHighConePosition - 0.03)
                        .alongWith(s_wrist.runToPosition(Wrist.kRightPosition)),
                    1.25)),
            s_intake.setVoltageCommand(-2.0))
        .withName("Auton-scoreConeHighStart");
  }

  public Command telescopeHome() {
    return s_telescope.runToPosition(Telescope.kHomePosition);
  }

  public Command scoreCubeMid() {
    return new SequentialCommandGroup(
            s_intake.setVoltageCommand(0.0),
            s_telescope.runToPosition(Telescope.kHomePosition),
            s_arm
                .runToPosition(Arm.kMidCubePosition)
                .alongWith(
                    s_arm.scheduleWhenArmGreater(
                        s_wrist.runToPosition(Wrist.kConeIntakePosition), Arm.kSafePosition)),
            s_intake.setVoltageCommand(2.5),
            new WaitCommand(0.3),
            s_intake.setVoltageCommand(0.0))
        .withName("Auton-scoreCubeMid");
  }

  public Command scoreCubeHigh() {
    return new SequentialCommandGroup(
            s_intake.setVoltageCommand(0.0),
            s_telescope.runToPosition(Telescope.kHomePosition),
            s_arm
                .runToPosition(Arm.kHighCubePosition)
                .alongWith(
                    s_arm.scheduleWhenArmGreater(
                        s_wrist.runToPosition(Wrist.kConeIntakePosition), Arm.kSafePosition),
                    s_arm.scheduleWhenArmGreater(
                        s_telescope.runToPosition(Telescope.kHighCubePosition),
                        Arm.kTelescopeClearPosition)),
            s_intake.setVoltageCommand(2.5),
            new WaitCommand(0.3),
            s_intake.setVoltageCommand(0.0),
            s_telescope.runToPosition(Telescope.kHomePosition))
        .withName("Auton-scoreCubeHigh");
  }

  public Command scoreConeMidThenHome() {
    return new SequentialCommandGroup(
            s_arm.runToPosition(Arm.kSafePosition),
            new ParallelCommandGroup(
                s_arm.runToPosition(Arm.kMidReleasePosition),
                s_wrist.runToPosition(Wrist.kRightPosition)),
            s_intake.setVoltageCommand(-8.0),
            s_arm.runToPosition(Arm.kSafePosition),
            s_wrist.runToPosition(Wrist.kConeIntakePosition),
            s_intake.setVoltageCommand(0.0),
            s_arm.runToPosition(Arm.kHomePosition))
        .withName("Auton-scoreConeMidThenHome");
  }

  public Command chuckCube() {
    return new SequentialCommandGroup(
        s_arm
            .runToPosition(Units.degreesToRadians(200))
            .alongWith(
                s_arm.scheduleWhenArmGreater(
                    s_intake.setVoltageCommand(15.0), Units.degreesToRadians(90.0))),
        s_arm.runToPosition(Arm.kSafePosition),
        s_intake.setVoltageCommand(0));
  }

  public Command intakeSafeCubeAfterScore() {
    return new ParallelCommandGroup(
            new InstantCommand(() -> s_telescope.setPosition(Telescope.kHomePosition)),
            s_intake.setVoltageCommand(Intake.kCubeInVoltageAuton),
            s_arm.runToPosition(Arm.kSafePosition),
            s_wrist.runToPosition(Wrist.kCubeIntakePosition))
        .withName("Auton-intakeCubeSafeAfterScore");
  }

  public Command intakeCubeAfterScoreFaster() {
    return new ParallelCommandGroup(
            new InstantCommand(() -> s_telescope.setPosition(Telescope.kHomePosition)),
            s_intake.setVoltageCommand(Intake.kCubeInVoltageAuton),
            s_arm.runToPosition(Arm.kCubeIntakePosition - Units.degreesToRadians(2)),
            s_wrist.runToPosition(Wrist.kCubeIntakePosition),
            s_arm.scheduleWhenArmLess(
                s_telescope.runToPosition(Telescope.kCubeIntakePosition + 0.015),
                Units.degreesToRadians(45)))
        .withName("Auton-intakeCubeAfterScoreFaster");
  }

  public Command preScoreCubeMid() {
    return new ParallelCommandGroup(
            s_telescope.runToPosition(Telescope.kHomePosition),
            s_arm.runToPosition(Arm.kMidCubePosition),
            s_wrist.runToPosition(Wrist.kConeIntakePosition))
        .withName("Auton-preScoreCubeMid");
  }

  public Command preScoreCubeHigh() {
    return new ParallelCommandGroup(
            s_arm.scheduleWhenArmGreater(
                s_telescope.runToPosition(Telescope.kHighCubePosition + 0.08),
                Units.degreesToRadians(135)),
            s_arm.runToPosition(Arm.kHighCubePosition),
            s_wrist.runToPosition(Wrist.kConeIntakePosition))
        .withName("Auton-preScoreCubeHigh");
  }

  public Command postScoreCube() {
    return s_intake
        .setVoltageCommand(5.0)
        .andThen(new WaitCommand(0.05))
        .withName("Auton-postScoreCube");
  }

  /*
   * Stop the intake and put arm and telescope directly to the safe position.
   * Don't mess with the wrist
   */
  public Command intakeStopQuick() {
    return new SequentialCommandGroup(
            s_arm.runToPosition(Arm.kSafePosition),
            s_intake.setVoltageCommand(0.0),
            s_telescope.runToPosition(Telescope.kHomePosition))
        .withName("Auton-intakeStopQuick");
  }

  public Command intakeStopOnGamepiece() {
    return Commands.run(() -> {})
        .until(s_intake::hasGamepiece)
        .andThen(Commands.print("Has Gamepiece"))
        .andThen(s_intake.setVoltageCommand(0))
        .withName("intakeStopOnGamepiece");
  }

  public HashMap<String, Command> get() {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("intakeCone", intakeCone());
    eventMap.put("intakeCube", intakeCube());
    eventMap.put("intakeConeAfterScore", intakeConeAfterScore());
    eventMap.put("intakeCubeAfterScore", intakeCubeAfterScore());
    eventMap.put("intakeCubeAfterScoreFaster", intakeCubeAfterScoreFaster());
    eventMap.put("intakeSafeCubeAfterScore", intakeSafeCubeAfterScore());
    eventMap.put("intakeStopQuick", intakeStopQuick());

    eventMap.put("intakeHome", intakeHome());
    eventMap.put("intakeHomeAfterScore", intakeHomeAfterScore());
    eventMap.put("intakeHomeCube", intakeHomeCube());
    eventMap.put("intakeSafeCube", intakeSafeCube());

    eventMap.put("scoreConeMid", scoreConeMid());
    eventMap.put("scoreConeHigh", scoreConeHigh());
    eventMap.put("scoreCubeMid", scoreCubeMid());
    eventMap.put("scoreCubeHigh", scoreCubeHigh());

    eventMap.put("intakeOnCube", intakeOnCube());
    eventMap.put("intakeOff", intakeOff());
    eventMap.put("intakeStopOnGamepiece", intakeStopOnGamepiece());

    eventMap.put("scoreConeMidThenHome", scoreConeMidThenHome());

    eventMap.put("preScoreCubeMid", preScoreCubeMid());
    eventMap.put("preScoreCubeHigh", preScoreCubeHigh());
    eventMap.put("postScoreCube", postScoreCube());
    return eventMap;
  }
}
