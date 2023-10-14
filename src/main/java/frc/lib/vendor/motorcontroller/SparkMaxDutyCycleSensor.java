package frc.lib.vendor.motorcontroller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import frc.lib.electromechanical.AbsoluteEncoder;

public class SparkMaxDutyCycleSensor implements AbsoluteEncoder {
  private final SparkMaxAbsoluteEncoder m_sensor;

  public SparkMaxDutyCycleSensor(CANSparkMax sparkMax) {
    m_sensor = sparkMax.getAbsoluteEncoder(Type.kDutyCycle);
  }

  @Override
  public double getPosition() {
    return m_sensor.getPosition();
  }

  @Override
  public void setPositionOffset(double position) {
    m_sensor.setZeroOffset(position);
  }

  public SparkMaxAbsoluteEncoder getSensor() {
    return m_sensor;
  }
}
