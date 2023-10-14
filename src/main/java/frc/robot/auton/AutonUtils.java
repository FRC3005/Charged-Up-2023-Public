package frc.robot.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.swerve.SwerveDrive;
import frc.robot.subsystems.robotstate.RobotState;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import org.tinylog.Logger;

public class AutonUtils {
  private static HashMap<String, PathPlannerTrajectory> m_cache = new HashMap<>();

  private static String cacheKey(String name, double maxVel, double maxAccel, boolean reversed) {
    return new StringBuilder()
        .append(name)
        .append(maxVel)
        .append(maxAccel)
        .append(reversed)
        .toString();
  }

  public static Pose2d initialPose(PathPlannerTrajectory trajectory) {
    return new Pose2d(
        trajectory.getInitialState().poseMeters.getTranslation(),
        trajectory.getInitialState().holonomicRotation);
  }

  public static PathPlannerTrajectory loadTrajectory(String name, double maxVel, double maxAccel) {
    return loadTrajectory(name, maxVel, maxAccel, false);
  }

  public static PathPlannerTrajectory loadTrajectory(
      String name, double maxVel, double maxAccel, boolean reversed) {
    String cacheKey_ = cacheKey(name, maxVel, maxAccel, reversed);
    if (m_cache.containsKey(cacheKey_)) {
      return m_cache.get(cacheKey_);
    }

    PathPlannerTrajectory trajectory = PathPlanner.loadPath(name, maxVel, maxAccel, reversed);

    if (trajectory == null) {
      Logger.tag("Auton Path").error("Failed to load trajectory: {}", name);
    }

    Logger.tag("Auton Path")
        .trace(
            "Loaded auton {}, max velocity: {}, max accel: {}, reversed: {}, starting state {}",
            name,
            maxVel,
            maxAccel,
            reversed,
            trajectory.getInitialState());

    /*
     * Store trajectories. This is done since multiple autons may load the same trajectories,
     * no need to load it again and again. If anything funky with trajectories is happening,
     * comment out the below. (e.g. I don't see any stored state in the trajectory, but it
     * is not clear if that is true, and if calling the trajectory stores any state.
     * Investigate later.)
     */
    m_cache.put(cacheKey_, trajectory);

    return trajectory;
  }

  /**
   * From BaseAutoBuilder Create a sequential command group that will follow each path in a path
   * group and trigger events as it goes. This will not run any stop events.
   *
   * @param pathGroup The path group to follow
   * @return Command for following all paths in the group
   */
  public static CommandBase followPathGroupWithEvents(
      List<PathPlannerTrajectory> pathGroup,
      Function<PathPlannerTrajectory, Command> pathFollowCommand,
      Map<String, Command> eventMap) {
    List<CommandBase> commands = new ArrayList<>();

    for (PathPlannerTrajectory path : pathGroup) {
      FollowPathWithEvents follow_command =
          new FollowPathWithEvents(pathFollowCommand.apply(path), path.getMarkers(), eventMap);
      commands.add(follow_command);
    }

    return Commands.sequence(commands.toArray(CommandBase[]::new));
  }

  public static Command flipFieldAndResetPose(
      SwerveDrive swerve, PathPlannerTrajectory trajectory) {
    return flipFieldAndResetPose(swerve, null, trajectory);
  }

  public static Command flipFieldAndResetPose(
      SwerveDrive swerve, RobotState robotState, PathPlannerTrajectory trajectory) {
    return new InstantCommand(
            () -> {
              PathPlannerTrajectory transformedTrajectory;
              transformedTrajectory =
                  PathPlannerTrajectory.transformTrajectoryForAlliance(
                      trajectory, DriverStation.getAlliance());

              swerve.resetOdometry(transformedTrajectory.getInitialHolonomicPose());

              if (robotState != null) {
                robotState.resetPoseEstimate(transformedTrajectory.getInitialHolonomicPose());
              }
            },
            swerve)
        .withName("Reset Odometry");
  }
}
