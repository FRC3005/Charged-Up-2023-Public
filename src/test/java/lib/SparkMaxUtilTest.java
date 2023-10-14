package lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxSim1;
import edu.wpi.first.hal.HAL;
import frc.lib.vendor.motorcontroller.SparkMax;
import frc.lib.vendor.motorcontroller.SparkMaxMonitor;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import org.junit.jupiter.api.*;

public class SparkMaxUtilTest {
  private static final double kEpsilon = 1E-9;

  @BeforeEach
  public void setup() {
    // Initialize HAL like this if using WPILib
    assert HAL.initialize(500, 0);
  }

  @AfterEach
  public void shutdown() throws Exception {
    HAL.shutdown();
  }

  @Test
  public void faultToStringTest() {
    short faults =
        (short)
            ((1 << CANSparkMax.FaultID.kBrownout.value)
                | (1 << CANSparkMax.FaultID.kDRVFault.value)
                | (1 << CANSparkMax.FaultID.kOtherFault.value));

    String result = SparkMaxUtils.faultWordToString(faults).strip();
    Assertions.assertEquals("kBrownout kDRVFault kOtherFault", result);
  }

  @Test
  public void monitorTest() {
    SparkMax sparkMax = new SparkMax(59);
    CANSparkMaxSim1 sparkMaxSim = new CANSparkMaxSim1(sparkMax);
    SparkMaxMonitor monitor = new SparkMaxMonitor();
    monitor.add(sparkMax);
    sparkMaxSim.setFaults(CANSparkMax.FaultID.kIWDTReset, CANSparkMax.FaultID.kCANRX);
    monitor.periodic();
  }
}
