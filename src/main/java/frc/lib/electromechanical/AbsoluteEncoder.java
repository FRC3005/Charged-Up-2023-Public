package frc.lib.electromechanical;

import frc.lib.telemetry.TelemetryBuilder;
import frc.lib.telemetry.TelemetryNode;

public interface AbsoluteEncoder extends TelemetryNode {
  public double getPosition();

  public void setPositionOffset(double position);

  public default boolean isConnected() {
    return true;
  }

  @Override
  default void bind(TelemetryBuilder builder) {
    builder.addDoubleProperty("Position", () -> this.getPosition(), null);
  }
}
