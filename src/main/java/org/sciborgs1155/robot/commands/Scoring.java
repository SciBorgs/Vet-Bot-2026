package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.commands.ScoringConstants.RELEASE_TIME;
import static org.sciborgs1155.robot.commands.ScoringConstants.RELEASE_VOLTAGE;
import static org.sciborgs1155.robot.commands.ScoringConstants.SCORE_HEIGHT;
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
   * Raises the elevator to {@code height}, spins the hold rollers out once it's there, then lowers
   * the elevator. Ends on its own once the elevator is back down.
   *
   * @param height Elevator height to release the ball at, clamped to the elevator's range.
   * @param releaseTime How long to spin the rollers out.
   * @param releaseVoltage How hard to spin the rollers out.
   * @return The scoring command.
   */
  public Command score(Distance height, Time releaseTime, Voltage releaseVoltage) {
    double goal = MathUtil.clamp(height.in(Meters), MIN_HEIGHT.in(Meters), MAX_HEIGHT.in(Meters));
    double bottom = MIN_HEIGHT.in(Meters);

    return elevator
        .goTo(goal)
        .withDeadline(
            Commands.waitUntil(() -> elevator.atPosition(goal))
                .andThen(hold.out(releaseVoltage.in(Volts)).withTimeout(releaseTime.in(Seconds))))
        .andThen(
            elevator.retract().until(() -> elevator.atPosition(bottom)).deadlineFor(hold.stop()))
        .withName("score");
  }
}
