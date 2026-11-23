package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import org.sciborgs1155.robot.Robot;

public class Pivot extends SubsystemBase implements AutoCloseable {
  private final PivotIO hardware;
  private final SysIdRoutine sysIdRoutine;

  private final ProfiledPIDController pid = new ProfiledPIDController(P, I, D, CONSTRAINTS);
  private final ArmFeedforward ff = new ArmFeedforward(S, G, V, A);

  /**
   * Creates a new Pivot subsystem.
   *
   * @param hardware the hardware implementationg of the Pivot subsystem
   */
  public Pivot(PivotIO hardware) {
    this.hardware = hardware;

    pid.setTolerance(POSITION_TOLERANCE.in(Radians));
    pid.reset(hardware.position());
    pid.setGoal(START_ANGLE.in(Radians));

    sysIdRoutine =
        new SysIdRoutine(
            new Config(
                RAMP_RATE,
                STEP_VOLTAGE,
                TIME_OUT,
                (state) -> SignalLogger.writeString("slapdown state", state.toString())),
            new Mechanism(voltage -> hardware.setVoltage(voltage.in(Volts)), null, this));
    SmartDashboard.putData(
        "Robot/slapdown/quasistatic forward",
        sysIdRoutine
            .quasistatic(Direction.kForward)
            // .until(() -> atPosition(MAX_ANGLE.in(Radians)))
            .withName("slapdown quasistatic forward"));
    SmartDashboard.putData(
        "Robot/slapdown/quasistatic backward",
        sysIdRoutine
            .quasistatic(Direction.kReverse)
            // .until(() -> atPosition(MIN_ANGLE.in(Radians)))
            .withName("slapdown quasistatic backward"));
    SmartDashboard.putData(
        "Robot/slapdown/dynamic forward",
        sysIdRoutine
            .dynamic(Direction.kForward)
            // .until(() -> atPosition(MAX_ANGLE.in(Radians)))
            .withName("slapdown dynamic forward"));
    SmartDashboard.putData(
        "Robot/slapdown/dynamic backward",
        sysIdRoutine
            .dynamic(Direction.kReverse)
            // .until(() -> atPosition(MIN_ANGLE.in(Radians)))
            .withName("slapdown dynamic backward"));
    setDefaultCommand(run(() -> hardware.setVoltage(0)));
  }

  /**
   * Returns the current position of the pivot in radians.
   * @return the current position in radians
   */
  public double position() {
    return hardware.position();
  }

  /**
   * @return a new Pivot subsystem with real motors
   */
  public static Pivot create() {
    return new Pivot(Robot.isReal() ? new RealPivot() : new SimPivot());
  }

  /**
   * @return a new Pivot subsystem with no motors
   */
  public static Pivot none() {
    return new Pivot(new NoPivot());
  }

  /**
   * Computes the voltage to apply to the pivot motor to move to the given angle.
   * @param angle the angle to move to in radians
   */
  public void update(double angle) {
    double rads = MathUtil.clamp(angle, MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians));
    double pidVoltage = pid.calculate(hardware.position(), rads);
    double ffVoltage = ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);
    hardware.setVoltage(pidVoltage + ffVoltage);
  }

  /**
   * A command factory to go to a given angle.
   * @param angle the angle to move to in radians.
   * @return a run command to move the pivot to the given angle.
   */
  public Command goTo(double angle) {
    return run(() -> update(angle));
  }

  /**
   * @return Returns a command to extend the pivot to max angle.
   */
  public Command extend() {
    return goTo(MAX_ANGLE.in(Radians));
  }

  /**
   * @return Returns a command to retract the pivot to min angle.
   */
  public Command retract() {
    return goTo(MIN_ANGLE.in(Radians));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
