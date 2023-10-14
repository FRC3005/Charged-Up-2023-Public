package lib;

import frc.lib.util.Faults;
import org.junit.jupiter.api.*;

public class FaultsTest {
  private static final double kEpsilon = 1E-9;

  @BeforeEach
  public void setup() {}

  @AfterEach // this method will run after each test
  public void shutdown() throws Exception {}

  private static String expectedString = "";
  private static Boolean fatalFault = false;
  private static int calls = 0;

  private static void runTest(String message, Boolean fatal) {
    Assertions.assertEquals(expectedString, message);
    Assertions.assertEquals(fatalFault, fatal);
    calls++;
  }

  @Test
  public void exampleTest() {
    expectedString = "Some message";
    fatalFault = false;
    Faults.onFault(FaultsTest::runTest);
    Faults.subsystem("Arm").error(expectedString);

    Assertions.assertEquals(1, Faults.count());
    Assertions.assertEquals(1, calls);

    Faults.subsystem("Arm").clear();
    Assertions.assertEquals(0, Faults.count());

    Faults.subsystem("Arm").error(expectedString);
    Faults.subsystem("Arm").error(expectedString);

    Assertions.assertEquals(1, Faults.count());
    Assertions.assertEquals(3, calls);

    Faults.clearAll();
    Assertions.assertEquals(0, Faults.count());

    expectedString = "Something else";
    fatalFault = false;
    Faults.subsystem("Arm").error(expectedString);
    expectedString = "also something else";
    fatalFault = true;
    Faults.subsystem("Leg").fatal(expectedString);
    Assertions.assertEquals(5, calls);
    Assertions.assertEquals(2, Faults.count());
    Faults.clearAll();
    Assertions.assertEquals(1, Faults.count());
  }
}
