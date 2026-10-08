package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.HOMING_VOLTAGE;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.KA;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_ACCEL;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_HEIGHT;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_VELOCITY;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MIN_HEIGHT;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.VELOCITY_TOLERANCE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.Robot;

public final class Elevator extends SubsystemBase implements AutoCloseable {
  private final ElevatorIO hardware;
  private final Timer stall = new Timer();

  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          KP,
          KI,
          KD,
          new TrapezoidProfile.Constraints(
              MAX_VELOCITY.in(MetersPerSecond), MAX_ACCEL.in(MetersPerSecondPerSecond)));
  private final ElevatorFeedforward ff = new ElevatorFeedforward(KS, KG, KV, KA);

  /** create new elevator based on mode */
  public static Elevator create() {
    return new Elevator(Robot.isReal() ? new RealElevator() : new SimElevator());
  }

  /** creates elevator using noelevator */
  public static Elevator none() {
    return new Elevator(new NoElevator());
  }

  /** Elevator Constructer */
  private Elevator(ElevatorIO hardware) {
    this.hardware = hardware;
  }

  private void update(double position) {
    double goal =
        Double.isNaN(position)
            ? MIN_HEIGHT.in(Meters)
            : MathUtil.clamp(position, MIN_HEIGHT.in(Meters), MAX_HEIGHT.in(Meters));
    double lastVelocity = pid.getSetpoint().velocity;
    double feedback = pid.calculate(hardware.position(), goal);
    double feedforward = ff.calculateWithVelocities(lastVelocity, pid.getSetpoint().velocity);

    hardware.setVoltage(feedforward + feedback);
  }

  /** returns velocity */
  public double velocity() {
    return hardware.velocity();
  }

  /** returns whether the elevator is at desired point / desired amount of extension */
  public boolean atGoal() {
    return pid.atGoal();
  }

  /** commands motors to run until a certain height is met */
  public Command goTo(DoubleSupplier height) {
    return run(() -> update(height.getAsDouble())).finallyDo(() -> hardware.setVoltage(0));
  }

  /** commands motors to go to set height */
  public Command goTo(double height) {
    return goTo(() -> height);
  }

  /** commands motors to run until the elevator is at its lowest point */
  public Command retract() {
    return goTo(MIN_HEIGHT.in(Meters));
  }

  /** commands motors to run until elevator is at its highest point */
  public Command extend() {
    return goTo(MAX_HEIGHT.in(Meters));
  }

  /**
   * runs the motor until the elevator is at its lowest extension point and sets that to its
   * starting position , makes movement from that point more accurate since it has a "perfect" point
   * of reference
   */
  public Command homingSequence() { // robot init
    return run(() -> {
          hardware.setVoltage(HOMING_VOLTAGE);
          if (Math.abs(hardware.velocity()) < VELOCITY_TOLERANCE) {
            if (!stall.isRunning()) stall.restart();
          } else {
            stall.stop();
          }
        })
        .until(
            () -> stall.hasElapsed(0.1)) // make stall longer if it stops as soon as sequence starts
        .andThen(
            () -> {
              hardware.setVoltage(0);
              hardware.resetPosition();
            });
  }

  /** every period the value of position/velocity updates in smartdashboard for reference */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator/Position", hardware.position());
    SmartDashboard.putNumber("Elevator/Velocity", hardware.velocity());
  }

  /** stops hardware */
  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
