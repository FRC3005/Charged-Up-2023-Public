package frc.lib.testmode;

public interface TestableSubsystem {
  public void testModePeriodic(boolean globalEnable);

  public boolean selfTest();
}
