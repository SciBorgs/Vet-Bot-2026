package org.sciborgs1155.robot;

import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.sciborgs1155.robot.elevator.Elevator;

class ElevatorTest {
  private Elevator elevator;

  @BeforeEach
  private void setup() {
    setupTests();
    elevator = Elevator.create();
    elevator.resetEncoders();
  }

  @AfterEach
  public void destroy() throws Exception {
    reset(elevator);
  }
}
