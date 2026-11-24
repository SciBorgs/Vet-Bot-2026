package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.commands.ScoringConstants.SCORE_HEIGHT;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_HEIGHT;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MIN_HEIGHT;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.lib.SimpleMotor;
import org.sciborgs1155.robot.commands.Scoring;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.endeffectorhold.EndEffectorHold;

class ScoringTest {
  private static final int MAX_TICKS = 500; // 10 s
  private static final double HEIGHT_TOLERANCE = 0.05;
  private static final double OVERSHOOT_ALLOWANCE = 0.1;

  private double holdVoltage;

  private Elevator elevator;
  private EndEffectorHold hold;
  private Scoring scoring;

  @BeforeEach
  void setup() {
    setupTests();
    elevator = Elevator.create();
    hold =
        new EndEffectorHold(
            new SimpleMotor(v -> holdVoltage = v * 12, v -> holdVoltage = v, () -> {}));
    scoring = new Scoring(elevator, hold);
  }

  @AfterEach
  void destroy() throws Exception {
    reset(elevator, hold);
  }

  /** Runs the command to completion (or the tick cap) and returns the highest elevator position. */
  private double runAndTrackMax(Command score) {
    CommandScheduler.getInstance().schedule(score);
    double maxHeight = elevator.position();
    for (int i = 0; i < MAX_TICKS && score.isScheduled(); i++) {
      fastForward(1);
      maxHeight = Math.max(maxHeight, elevator.position());
    }
    return maxHeight;
  }

  private static void assertReached(double goal, double maxHeight) {
    assertTrue(maxHeight > goal - HEIGHT_TOLERANCE, "only reached " + maxHeight);
    assertTrue(maxHeight < goal + OVERSHOOT_ALLOWANCE, "overshot to " + maxHeight);
  }

  @Test
  void scoreHeightInRange() {
    assertTrue(SCORE_HEIGHT.in(Meters) > MIN_HEIGHT.in(Meters));
    assertTrue(SCORE_HEIGHT.in(Meters) <= MAX_HEIGHT.in(Meters));
  }

  @Test
  void scoreFinishesAtBottom() {
    Command score = scoring.score();
    double maxHeight = runAndTrackMax(score);
    assertFalse(score.isScheduled(), "score never finished");
    assertReached(SCORE_HEIGHT.in(Meters), maxHeight);
    assertTrue(elevator.atPosition(MIN_HEIGHT.in(Meters)));
  }

  @Test
  void scoreCustomHeight() {
    Command score = scoring.score(Meters.of(0.8));
    double maxHeight = runAndTrackMax(score);
    assertFalse(score.isScheduled(), "score never finished");
    assertReached(0.8, maxHeight);
    assertTrue(elevator.atPosition(MIN_HEIGHT.in(Meters)));
  }

  @Test
  void releasesOnlyAtHeight() {
    Command score = scoring.score();
    CommandScheduler.getInstance().schedule(score);
    boolean released = false;
    for (int i = 0; i < MAX_TICKS && score.isScheduled(); i++) {
      fastForward(1);
      if (holdVoltage < 0) {
        released = true;
        assertReached(SCORE_HEIGHT.in(Meters), elevator.position());
      }
    }
    assertTrue(released, "rollers never released");
  }
}
