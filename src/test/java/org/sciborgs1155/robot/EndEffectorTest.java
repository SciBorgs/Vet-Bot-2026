package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.endeffectorpivot.EndEffectorPivotConstants.*;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Random;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.endeffectorpivot.EndEffectorPivot;

class EndEffectorTest {
  private EndEffectorPivot endEffector;
  private final Random random = new Random();

  /** set up tests and create the end effector */
  @BeforeEach
  private void setup() {
    endEffector = EndEffectorPivot.create();
    setupTests();
  }

  /**
   * @throws Exception reset the end effector
   */
  @AfterEach
  void destroy() throws Exception {
    reset(endEffector);
  }

  /** test the movement of the end effector */
  @Test
  void orientation() {
    double setpoint = random.nextDouble(MIN_ANGLE, MAX_ANGLE);
    CommandScheduler.getInstance()
        .schedule(endEffector.goTo(setpoint).withDeadline(Commands.waitSeconds(7)));
    fastForward(Seconds.of(6));
    assertEquals(setpoint, endEffector.position(), POSITION_TOLERANCE.in(Radians));
  }
}
