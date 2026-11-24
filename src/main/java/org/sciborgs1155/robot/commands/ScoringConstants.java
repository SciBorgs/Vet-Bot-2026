package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
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

  private ScoringConstants() {}
}
