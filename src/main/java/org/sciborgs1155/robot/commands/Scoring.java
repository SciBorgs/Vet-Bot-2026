package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.commands.ScoringConstants.MIN_RELEASE_TIME;
import static org.sciborgs1155.robot.commands.ScoringConstants.MIN_SCORE_HEIGHT;
import static org.sciborgs1155.robot.commands.ScoringConstants.RAISE_TIMEOUT;
import static org.sciborgs1155.robot.commands.ScoringConstants.RELEASE_TIME;
import static org.sciborgs1155.robot.commands.ScoringConstants.RELEASE_VOLTAGE;
import static org.sciborgs1155.robot.commands.ScoringConstants.SCORE_HEIGHT;
import static org.sciborgs1155.robot.commands.ScoringConstants.SETTLE_VELOCITY;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_HEIGHT;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MIN_HEIGHT;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.endeffectorhold.EndEffectorHold;

/** Dunks the ball: raise the elevator, let the end effector release the ball, lower again. */
public class Scoring {
  @NotLogged private final Elevator elevator;
  @NotLogged private final EndEffectorHold hold;

  @NotLogged
  private final DoubleEntry heightEntry =
      Tuning.entry("/Robot/tuning/scoring/height (m)", SCORE_HEIGHT.in(Meters));

  @NotLogged
  private final DoubleEntry releaseTimeEntry =
      Tuning.entry("/Robot/tuning/scoring/release time (s)", RELEASE_TIME.in(Seconds));

  @NotLogged
  private final DoubleEntry releaseVoltageEntry =
      Tuning.entry("/Robot/tuning/scoring/release voltage (V)", RELEASE_VOLTAGE.in(Volts));

  /**
   * Constructor for a Scoring command object.
   *
   * @param elevator The elevator carrying the end effector.
   * @param hold The end effector rollers holding the ball.
   */
  public Scoring(Elevator elevator, EndEffectorHold hold) {
    this.elevator = elevator;
    this.hold = hold;
  }

  /**
   * Scores with the values currently on the dashboard (defaults from {@link ScoringConstants}).
   * Values are read when the command starts.
   *
   * @return The scoring command.
   */
  public Command score() {
    return Commands.defer(
            () ->
                score(
                    Meters.of(heightEntry.get()),
                    Seconds.of(releaseTimeEntry.get()),
                    Volts.of(releaseVoltageEntry.get())),
            Set.of(elevator, hold))
        .withName("score");
  }

  /**
   * Scores at a custom height with the default release time and voltage.
   *
   * @param height Elevator height to release the ball at.
   * @return The scoring command.
   */
  public Command score(Distance height) {
    return score(height, RELEASE_TIME, RELEASE_VOLTAGE);
  }

  /**
   * Raises the elevator to {@code height}, spins the hold rollers out once the carriage has settled
   * there, then lowers the elevator. If it doesn't settle within {@link
   * ScoringConstants#RAISE_TIMEOUT}, it lowers without releasing. Ends on its own once the elevator
   * is back down.
   *
   * @param height Elevator height to release the ball at, clamped to [MIN_SCORE_HEIGHT,
   *     MAX_HEIGHT].
   * @param releaseTime How long to spin the rollers out, at least MIN_RELEASE_TIME.
   * @param releaseVoltage How hard to spin the rollers out.
   * @return The scoring command.
   */
  public Command score(Distance height, Time releaseTime, Voltage releaseVoltage) {
    double goal =
        MathUtil.clamp(height.in(Meters), MIN_SCORE_HEIGHT.in(Meters), MAX_HEIGHT.in(Meters));
    double releaseSeconds = Math.max(releaseTime.in(Seconds), MIN_RELEASE_TIME.in(Seconds));
    double bottom = MIN_HEIGHT.in(Meters);
    BooleanSupplier settled =
        () ->
            elevator.atPosition(goal)
                && Math.abs(elevator.velocity()) < SETTLE_VELOCITY.in(MetersPerSecond);

    Command release = hold.out(releaseVoltage.in(Volts)).withTimeout(releaseSeconds);

    return elevator
        .goTo(goal)
        .withDeadline(
            Commands.waitUntil(settled)
                .withTimeout(RAISE_TIMEOUT.in(Seconds))
                .andThen(Commands.either(release, Commands.none(), settled)))
        .andThen(
            elevator.retract().until(() -> elevator.atPosition(bottom)).deadlineFor(hold.stop()))
        .withName("score");
  }
}
