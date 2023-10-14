// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.function.Supplier;

public class DriveToRampCommand extends CommandBase {
  private DriveSubsystem m_driveSubsystem;
  private PIDController m_xController;
  private PIDController m_yController;
  private PIDController m_thetaController;
  private PathConstraints m_containts;
  private Supplier<Pose2d> m_poseSupplier;
  private Command m_innerCommand = null;
  private static final double kCenterOfRampX = 3.92;
  private static final double kExtraXDistance = 1.4; // 0.6m = ~2ft

  public DriveToRampCommand(
      DriveSubsystem drive,
      PIDController xController,
      PIDController yController,
      PIDController thetaController,
      PathConstraints constraints,
      Supplier<Pose2d> poseSupplier) {
    addRequirements(drive);
    m_driveSubsystem = drive;
    m_xController = xController;
    m_yController = yController;
    m_thetaController = thetaController;
    m_containts = constraints;
    m_poseSupplier = poseSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d pose = m_poseSupplier.get();
    ChassisSpeeds chassisSpeed = m_driveSubsystem.getChassisSpeeds();

    /*
     * Target just beyond the center of the ramp for the initial
     * drive portion. This will depend on the side of the field the
     * robot is currently on.
     */
    double xTarget = 0.0;
    if (pose.getX() > kCenterOfRampX) {
      xTarget = kCenterOfRampX - kExtraXDistance;
    } else {
      xTarget = kCenterOfRampX + kExtraXDistance;
    }

    /*
     * Set the heading to be square to the ramp allowing either 0
     * or 180 degrees
     */
    Rotation2d thetaTarget;
    if (Math.abs(pose.getRotation().getDegrees()) > 45.0) {
      thetaTarget = Rotation2d.fromDegrees(180);
    } else {
      thetaTarget = new Rotation2d(0.0);
    }

    double trajStarting =
        Math.sqrt(
            chassisSpeed.vxMetersPerSecond * chassisSpeed.vxMetersPerSecond
                + chassisSpeed.vyMetersPerSecond * chassisSpeed.vyMetersPerSecond);
    PathPlannerTrajectory traj =
        PathPlanner.generatePath(
            m_containts,
            new PathPoint(
                new Translation2d(pose.getX(), pose.getY()), pose.getRotation(), trajStarting),
            // TODO: Add a point to drive in front of the ramp
            new PathPoint(new Translation2d(xTarget, pose.getY()), thetaTarget, 0));

    m_innerCommand =
        m_driveSubsystem.trajectoryFollowerCommand(
            traj, m_xController, m_yController, m_thetaController);
    m_innerCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_innerCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_innerCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_innerCommand.isFinished();
  }
}
