package lib;

import com.revrobotics.CANSparkMaxSim1;
import edu.wpi.first.hal.HAL;
import frc.lib.controller.Controller;
import frc.lib.controller.PIDGains;
import frc.lib.vendor.motorcontroller.SparkMax;
import org.junit.jupiter.api.*;

public class SparkMaxControllerTest {
  private static final double kEpsilon = 1E-9;

  @BeforeEach // this method will run before each test
  public void setup() {
    // Initialize HAL like this if using WPILib
    assert HAL.initialize(500, 0);
  }

  @AfterEach // this method will run after each test
  public void shutdown() throws Exception {
    HAL.shutdown();
  }

  @Test
  public void controllerWrapTest() {
    SparkMax sparkMax = new SparkMax(1);
    Controller controller = sparkMax.positionController(new PIDGains());
    CANSparkMaxSim1 sim = new CANSparkMaxSim1(sparkMax);

    controller.enableContinuousInput(-Math.PI, Math.PI);

    controller.setReference(Math.PI, (Math.PI / 2) * 4 * Math.PI);

    // Assertions.assertEquals("Unwrapped value", 5 * Math.PI, sim.getSetpoint(), kEpsilon);
  }
}
