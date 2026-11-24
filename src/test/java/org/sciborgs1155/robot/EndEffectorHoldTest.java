package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.run;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.lib.SimpleMotor;
import org.sciborgs1155.robot.endeffectorhold.EndEffectorHold;

class EndEffectorHoldTest {
  private double lastVoltage;
  private EndEffectorHold hold;

  @BeforeEach
  void setup() {
    setupTests();
    hold = new EndEffectorHold(new SimpleMotor(v -> {}, v -> lastVoltage = v, () -> {}));
  }

  @AfterEach
  void destroy() throws Exception {
    reset(hold);
  }

  @Test
  void outUsesGivenVoltage() {
    run(hold.out(3.0));
    assertEquals(-3.0, lastVoltage, 1e-9);
  }

  @Test
  void outIgnoresSign() {
    run(hold.out(-3.0));
    assertEquals(-3.0, lastVoltage, 1e-9);
  }
}
