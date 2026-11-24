package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

/** Defaults for {@link Scoring}. All can be overridden per call or from the dashboard. */
public final class ScoringConstants {
  /** Elevator height to dunk from. */
  public static final Distance SCORE_HEIGHT = Meters.of(1.2); // TODO: measure

  /** How long the hold rollers spin out to let go of the ball. */
  public static final Time RELEASE_TIME = Seconds.of(0.5); // TODO: tune

  /** How hard the hold rollers spin out. */
  public static final Voltage RELEASE_VOLTAGE = Volts.of(3); // TODO: tune

  /**
   * Lowest height score will release at, so a bad dashboard value can't spit the ball on the floor.
   */
  public static final Distance MIN_SCORE_HEIGHT = Meters.of(0.3);

  /** Shortest release, so a zero or negative release time still lets go of the ball. */
  public static final Time MIN_RELEASE_TIME = Seconds.of(0.1);

  /** The carriage must be slower than this before the ball is released. */
  public static final LinearVelocity SETTLE_VELOCITY = MetersPerSecond.of(0.05);

  /** If the elevator isn't settled at height by then, give up without releasing and lower. */
  public static final Time RAISE_TIMEOUT = Seconds.of(3); // TODO: tune

  private ScoringConstants() {}
}
