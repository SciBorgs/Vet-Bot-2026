package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.HOMING_VOLTAGE;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_ACCEL;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_HEIGHT;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_VELOCITY;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MIN_HEIGHT;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.VELOCITY_TOLERANCE;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kA;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kD;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kG;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kI;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kP;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kS;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kV;

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

public class Elevator extends SubsystemBase implements AutoCloseable {
  private final ElevatorIO hardware;

  public static Elevator create() {
    return new Elevator(Robot.isReal() ? new RealElevator() : new SimElevator());
  }

  public static Elevator none() {
    return new Elevator(new NoElevator());
  }

  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          kP,
          kI,
          kD,
          new TrapezoidProfile.Constraints(
              MAX_VELOCITY.in(MetersPerSecond), MAX_ACCEL.in(MetersPerSecondPerSecond)));
  private final ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV, kA);

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

  public double velocity() {
    return hardware.velocity();
  }

  public boolean atGoal() {
    return pid.atGoal();
  }

  public Command goTo(DoubleSupplier height) {
    return run(() -> update(height.getAsDouble())).finallyDo(() -> hardware.setVoltage(0));
  }

  public Command goTo(double height) {
    return goTo(() -> height);
  }

  public Command retract() {
    return goTo(MIN_HEIGHT.in(Meters));
  }

  public Command extend() {
    return goTo(MAX_HEIGHT.in(Meters));
  }

  private final Timer stall = new Timer();

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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator/Position", hardware.position());
    SmartDashboard.putNumber("Elevator/Velocity", hardware.velocity());
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
