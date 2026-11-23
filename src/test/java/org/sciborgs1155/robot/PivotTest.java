package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Random;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.pivot.Pivot;

public class PivotTest {
  private Pivot pivot;
  private final Random random = new Random();

  @BeforeEach
  private void setup() {
    pivot = pivot.create();
    setupTests();
  }

  /**
   * Resets the pivot object.
   *
   * @throws Exception
   */
  @AfterEach
  public void destroy() throws Exception {
    reset(pivot);
  }

  /** Tests to see if the intake pivot works as normal. */
  @Test
  public void orientation() {
    double setpoint = random.nextDouble(MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians));
    CommandScheduler.getInstance()
        .schedule(pivot.goTo(setpoint).withDeadline(Commands.waitSeconds(7)));
    fastForward(Seconds.of(6));
    assertEquals(setpoint, pivot.position(), POSITION_TOLERANCE.in(Radians));
  }
}
