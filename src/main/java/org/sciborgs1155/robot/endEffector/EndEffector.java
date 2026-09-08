package org.sciborgs1155.robot.endEffector;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.endEffector.EndEffectorConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Robot;

public class EndEffector extends SubsystemBase implements AutoCloseable {
  private final EndEffectorIO hardware;

  private final ProfiledPIDController pid =
      new ProfiledPIDController(kP, kI, kD, new Constraints(MAX_VELOCITY, MAX_ACCEL));

  private final ArmFeedforward ff = new ArmFeedforward(kS, kG, kV, kA);

  @NotLogged
  private final DoubleEntry tuningP = Tuning.entry("Robot/tuning/endEffector/tuningP", kP);

  @NotLogged
  private final DoubleEntry tuningI = Tuning.entry("Robot/tuning/endEffector/tuningI", kI);

  @NotLogged
  private final DoubleEntry tuningD = Tuning.entry("Robot/tuning/endEffector/tuningD", kD);

  @NotLogged
  private final DoubleEntry tuningS = Tuning.entry("Robot/tuning/endEffector/tuningS", kS);

  @NotLogged
  private final DoubleEntry tuningG = Tuning.entry("Robot/tuning/endEffector/tuningG", kG);

  @NotLogged
  private final DoubleEntry tuningV = Tuning.entry("Robot/tuning/endEffector/tuningV", kV);

  @NotLogged
  private final DoubleEntry tuningA = Tuning.entry("Robot/tuning/endEffector/tuningA", kA);

  private final SysIdRoutine sysIdRoutine;

  public static EndEffector create() {
    return new EndEffector(Robot.isReal() ? new RealEndEffector() : new SimEndEffector());
  }

  public static EndEffector none() {
    return new EndEffector(new NoEndEffector());
  }

  public EndEffector(EndEffectorIO hardware) {
    this.hardware = hardware;

    pid.setTolerance(POSITION_TOLERANCE.in(Radians));
    pid.reset(hardware.position());
    pid.setGoal(MAX_ANGLE.in(Radians));

    sysIdRoutine =
        new SysIdRoutine(
            new Config(
                RAMP_RATE,
                STEP_VOLTS,
                TIME_OUT,
                (state) -> SignalLogger.writeString("end effector state", state.toString())),
            new Mechanism(voltage -> hardware.setVoltage(voltage.in(Volts)), null, this));

    SmartDashboard.putData(
        "Robot/endEffector.quasistatic forward",
        sysIdRoutine.quasistatic(Direction.kForward).withName("end effector quasistatic forward"));
    SmartDashboard.putData(
        "Robot/endEffector.quasistatic backward",
        sysIdRoutine.quasistatic(Direction.kReverse).withName("end effector quasistatic backward"));
    SmartDashboard.putData(
        "Robot/endEffector.dynamic forward",
        sysIdRoutine.dynamic(Direction.kForward).withName("end effector dynamic forward"));
    SmartDashboard.putData(
        "Robot/endEffector.dynamic backward",
        sysIdRoutine.dynamic(Direction.kReverse).withName("end effector dynamic backward"));
  }

  public void update(double angle, ProfiledPIDController pid) {
    double rads = MathUtil.clamp(angle, MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians));
    double pidV = pid.calculate(position(), rads);
    double ffV = ff.calculate(setpoint(), pid.getSetpoint().velocity);
    hardware.setVoltage(pidV + ffV);
  }

  public void update(double angle) {
    update(angle, pid);
  }

  public boolean atPosition(double angle) {
    return Math.abs(angle - position()) < POSITION_TOLERANCE.in(Radians);
  }

  public Command goTo(double angle) {
    return run(() -> update(angle)).withName("go to angle");
  }

  public Command goTo(DoubleSupplier angle) {
    return run(() -> update(angle.getAsDouble())).withName("go to angle");
  }

  public Command open() {
    return goTo(0.8);
  }

  public Command closeEffector() {
    return goTo(0.2);
  }

  @Logged
  public double position() {
    return hardware.position();
  }

  @Logged
  public double setpoint() {
    return pid.getSetpoint().position;
  }

  @Override
  public void periodic() {
    if (TUNING) {
      pid.setP(tuningP.get());
      pid.setI(tuningI.get());
      pid.setD(tuningD.get());
      ff.setKg(tuningG.get());
      ff.setKa(tuningA.get());
      ff.setKv(tuningV.get());
      ff.setKs(tuningS.get());
    }
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
