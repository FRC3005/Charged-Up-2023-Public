package frc.lib.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BiConsumer;
import org.tinylog.Logger;

public class Faults {
  private static HashMap<String, Fault> s_faults = new HashMap<>();
  protected static List<BiConsumer<String, Boolean>> s_callbacks = new ArrayList<>();

  private Faults() {}

  public static int count() {
    int cnt = 0;
    for (var fault : s_faults.values()) {
      if (fault.m_message.length() > 0) {
        cnt++;
      }
    }
    return cnt;
  }

  public static void clearAll() {
    for (var fault : s_faults.values()) {
      fault.clear();
    }
  }

  public static class Fault {
    private String m_message;
    private boolean m_fatal = false;
    private String m_tag = "";

    public Fault(String tag) {
      m_tag = tag;
    }

    public void fatal(String message) {
      m_fatal = true;
      m_message = message;
      Logger.tag(m_tag).error(message);

      for (var callback : Faults.s_callbacks) {
        callback.accept(m_message, true);
      }
    }

    public void error(String message) {
      if (!m_fatal) {
        m_message = message;
      }
      ;
      Logger.tag(m_tag).error(message);
      for (var callback : Faults.s_callbacks) {
        callback.accept(m_message, false);
      }
    }

    public void clear() {
      if (!m_fatal) {
        m_message = "";
      }
    }

    public String message() {
      return m_message;
    }
  }

  public static Fault subsystem(String subsystem) {
    if (s_faults.containsKey(subsystem)) {
      return s_faults.get(subsystem);
    }
    Fault fault = new Fault(subsystem);
    s_faults.put(subsystem, fault);
    return fault;
  }

  public static void onFault(BiConsumer<String, Boolean> callback) {
    s_callbacks.add(callback);
  }
}
