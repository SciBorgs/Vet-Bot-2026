package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.run;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.DISTANCE_TOLERANCE;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_HEIGHT;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MIN_HEIGHT;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.elevator.Elevator;

class ElevatorTest {
  private Elevator elevator;

  @BeforeEach
  void setup() {
    setupTests();
    elevator = Elevator.create();
  }

  @AfterEach
  void destroy() throws Exception {
    reset(elevator);
  }

  @Test
  void exceedClampTest() {
    double max = MAX_HEIGHT.in(Meters);
    double min = MIN_HEIGHT.in(Meters);

    Command up = elevator.goTo(max + 10);
    CommandScheduler.getInstance().schedule(up);

    for (int i = 0; i < 1155; i++) {
      fastForward(1);
      assertTrue(elevator.position() <= max, "elevator pos above max: " + elevator.position());
      assertTrue(elevator.setpoint() <= max, "setpoint above max : " + elevator.setpoint());
    }

    up.cancel();

    Command down = elevator.goTo(min - 10);
    CommandScheduler.getInstance().schedule(down);

    for (int i = 0; i < 1155; i++) {
      fastForward(1);
      assertTrue(elevator.position() >= min, "elevator pos below min: " + elevator.position());
      assertTrue(elevator.setpoint() >= min, "setpoint below min : " + elevator.setpoint());
    }
  }

  @Test
  void goToTest() {
    double goal = (MIN_HEIGHT.in(Meters) + MAX_HEIGHT.in(Meters)) / 2;
    Command goToTarget = elevator.goTo(goal);
    run(goToTarget, 250); // 250 tick * 20 ms = 5 s
    assertEquals(goal, elevator.position(), DISTANCE_TOLERANCE);
  }
}
