package frc.robot.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.telemetry.TelemetryNode;
import org.tinylog.Logger;

public abstract class AutonCommandBase extends SequentialCommandGroup implements TelemetryNode {
  private double m_autonStartTime = 0;

  public AutonCommandBase() {
    String name = this.getClass().getSimpleName();
    name = name.substring(name.lastIndexOf('.') + 1);

    Logger.tag("Auton").info("Adding Auton {}", name);
    AutonChooser.addAuton(this, name);
  }

  public AutonCommandBase(String name) {
    AutonChooser.addAuton(this, name);
  }

  public void addCommandsWithLog(String tag, Command... commands) {
    // Add logging to each command

    // Log start of group
    super.addCommands(
        new InstantCommand(
            () -> {
              m_autonStartTime = Timer.getFPGATimestamp();
              Logger.tag(tag).trace("Auton goup Starting at {}", m_autonStartTime);
            }));

    // Log start and end of each command
    for (var cmd : commands) {
      super.addCommands(
          new InstantCommand(
              () ->
                  Logger.tag(tag)
                      .trace(
                          "Starting command step {}, at {}",
                          cmd.getName(),
                          Timer.getFPGATimestamp())),
          cmd,
          new InstantCommand(
              () ->
                  Logger.tag(tag)
                      .trace(
                          "Ending command step {}, at {}",
                          cmd.getName(),
                          Timer.getFPGATimestamp())));
    }

    // Log end of auton
    super.addCommands(
        new InstantCommand(
            () -> {
              Logger.tag(tag)
                  .trace(
                      "Auton group complete after {} seconds",
                      Timer.getFPGATimestamp() - m_autonStartTime);
            }));
  }
}
