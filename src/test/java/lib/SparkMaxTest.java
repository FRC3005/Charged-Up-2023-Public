package lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.hal.HAL;
import frc.lib.vendor.motorcontroller.SparkMax;
import java.util.function.BiFunction;
import org.junit.jupiter.api.*;

public class SparkMaxTest {

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0);
  }

  @AfterEach
  public void shutdown() throws Exception {
    HAL.shutdown();
  }

  private static final BiFunction<CANSparkMax, Boolean, Boolean> initFunction =
      (CANSparkMax sparkMax, Boolean initializer) -> {
        sparkMax.setIdleMode(IdleMode.kBrake);
        sparkMax.setSmartCurrentLimit(20);

        sparkMax.enableVoltageCompensation(12.12);
        return true;
      };

  @Test
  public void basicDriverTest() {
    SparkMax sparkMax = new SparkMax(4, MotorType.kBrushless).withInitializer(initFunction);

    int devId = sparkMax.getDeviceId();
    var tmp = sparkMax.getIdleMode();

    Assertions.assertEquals(4, devId);

    Assertions.assertEquals(IdleMode.kBrake, tmp);

    sparkMax.mutate(
        (spark, init) -> {
          spark.setIdleMode(IdleMode.kCoast);
          return true;
        });

    tmp = sparkMax.getIdleMode();
    Assertions.assertEquals(IdleMode.kCoast, tmp);
  }
}
