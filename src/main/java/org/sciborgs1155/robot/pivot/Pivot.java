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

  public Pivot(PivotIO hardware) {
    this.hardware = hardware;

    pid.setTolerance(POSITION_TOLERANCE.in(Radians));
    pid.reset(hardware.position());
    pid.setGoal(START_ANGLE.in(Radians));

    // setDefaultCommand(nothing());

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

  public double position() {
    return hardware.position();
  }

  public static Pivot create() {
    return new Pivot(Robot.isReal() ? new RealPivot() : new SimPivot());
  }

  public static Pivot none() {
    return new Pivot(new NoPivot());
  }

  public void update(double angle) {
    double rads = MathUtil.clamp(angle, MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians));
    double pidVoltage = pid.calculate(hardware.position(), rads);
    double ffVoltage = ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);
    hardware.setVoltage(pidVoltage + ffVoltage);
  }

  public Command goTo(double angle) {
    return run(() -> update(angle));
  }

  public Command extend() {
    return goTo(MAX_ANGLE.in(Radians));
  }

  public Command retract() {
    return goTo(MIN_ANGLE.in(Radians));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
