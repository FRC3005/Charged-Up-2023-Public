package frc.robot.auton.locations;

public abstract class Location {
  // public abstract Pose2d get();

  public String getName() {
    String name = this.getClass().getSimpleName();
    return name.substring(name.lastIndexOf('.') + 1);
  }
}
