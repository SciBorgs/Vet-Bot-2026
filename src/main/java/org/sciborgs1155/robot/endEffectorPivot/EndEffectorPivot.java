package org.sciborgs1155.robot.endEffectorPivot;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Ports.EndEffector.*;
import static org.sciborgs1155.robot.endEffectorPivot.EndEffectorPivotConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.robot.Robot;

public class EndEffectorPivot extends SubsystemBase implements AutoCloseable {
  private final EndEffectorIO hardware;
  private final ProfiledPIDController pid =
      new ProfiledPIDController(kP, kI, kP, new Constraints(MAX_VELOCITY, MAX_ACCEL));
  private final ArmFeedforward ff = new ArmFeedforward(kS, kG, kV, kA);

  public EndEffectorPivot(EndEffectorIO hardware) {
    this.hardware = hardware;
    pid.setTolerance(POSITION_TOLERANCE.in(Radians));
    pid.reset(hardware.position());
    pid.setGoal(START_ANGLE);
    setDefaultCommand(stop());
  }

  public static EndEffectorPivot create() {
    return new EndEffectorPivot(Robot.isReal() ? new RealEndEffector() : new SimEndEffector());
  }

  public static EndEffectorPivot none() {
    return new EndEffectorPivot(new NoEndEffector());
  }

  public Command turn(double voltage) {
    return run(() -> hardware.setVoltage(voltage));
  }

  public Command stop() {
    return turn(0);
  }

  public void update(double angle, ProfiledPIDController pid) {
    double rads = MathUtil.clamp(angle, MIN_ANGLE, MAX_ANGLE);
    double pidVoltage = pid.calculate(hardware.position(), rads);
    double ffVoltage = ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);
    hardware.setVoltage(pidVoltage + ffVoltage);
  }

  public void update(double angle) {
    update(angle, pid);
  }

  public Command goTo(double angle) {
    return run(() -> update(angle)); // .withName("go to angle"));
  }

  public Command goTo(DoubleSupplier angle) {
    return run(() -> update(angle.getAsDouble())); // .withName("go to angle"));
  }

  public Command score() {
    return run(() -> goTo(SCORE_ANGLE));
  }

  @Logged
  public double position() {
    return hardware.position();
  }

  @Logged
  public double velocity() {
    return hardware.velocity();
  }

  @Logged
  public double positionSetpoint() {
    return pid.getSetpoint().position;
  }

  @Logged
  public double velocitySetpoint() {
    return pid.getSetpoint().velocity;
  }

  public boolean atPosition(double angle) {
    return Math.abs(angle - position()) < POSITION_TOLERANCE.in(Radians);
  }

  @Logged
  public boolean atGoal() {
    return pid.atGoal();
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
