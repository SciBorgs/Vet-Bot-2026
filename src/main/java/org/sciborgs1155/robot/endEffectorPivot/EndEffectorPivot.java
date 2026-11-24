package org.sciborgs1155.robot.endeffectorpivot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Ports.EndEffector.*;
import static org.sciborgs1155.robot.endeffectorpivot.EndEffectorPivotConstants.*;

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
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Robot;

public class EndEffectorPivot extends SubsystemBase implements AutoCloseable {
  private final EndEffectorIO hardware;
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          kP, kI, kD, new Constraints(MAX_VELOCITY.in(MetersPerSecond), MAX_ACCEL));

  @NotLogged
  private final DoubleEntry tuningP = Tuning.entry("Robot/tuning/endeffector/tuningP", kP);

  @NotLogged
  private final DoubleEntry tuningI = Tuning.entry("Robot/tuning/endeffector/tuningI", kI);

  @NotLogged
  private final DoubleEntry tuningD = Tuning.entry("Robot/tuning/endeffector/tuningD", kD);

  @NotLogged
  private final DoubleEntry tuningS = Tuning.entry("Robot/tuning/endeffector/tuningS", kS);

  @NotLogged
  private final DoubleEntry tuningG = Tuning.entry("Robot/tuning/endeffector/tuningG", kG);

  @NotLogged
  private final DoubleEntry tuningV = Tuning.entry("Robot/tuning/endeffector/tuningV", kV);

  @NotLogged
  private final DoubleEntry tuningA = Tuning.entry("Robot/tuning/endeffector/tuningA", kA);

  /** Routine for recording and analyzing motor data. */
  private final SysIdRoutine sysIdRoutine;

  private final ArmFeedforward ff = new ArmFeedforward(kS, kG, kV, kA);

  /**
   * @param hardware instantiates hardware
   */
  public EndEffectorPivot(EndEffectorIO hardware) {
    this.hardware = hardware;
    pid.setTolerance(POSITION_TOLERANCE.in(Radians));
    pid.reset(hardware.position());
    pid.setGoal(START_ANGLE);

    sysIdRoutine =
        new SysIdRoutine(
            new Config(
                RAMP_RATE,
                STEP_VOLTAGE,
                TIME_OUT,
                (state) -> SignalLogger.writeString("slapdown state", state.toString())),
            new Mechanism(voltage -> hardware.setVoltage(voltage.in(Volts)), null, this));
    SmartDashboard.putData(
        "Robot/endeffector/quasistaic forward",
        sysIdRoutine.quasistatic(Direction.kForward).withName("endeffector quasistaic forward"));
    SmartDashboard.putData(
        "Robot/endeffector/quasistaic backward",
        sysIdRoutine.quasistatic(Direction.kReverse).withName("endeffector quasistaic backward"));
    SmartDashboard.putData(
        "Robot/endeffector/dynamic forward",
        sysIdRoutine.dynamic(Direction.kForward).withName("endeffector dynamic forward"));
    SmartDashboard.putData(
        "Robot/endeffector/dynamic backward",
        sysIdRoutine.dynamic(Direction.kReverse).withName("endeffector dynamic backward"));

    setDefaultCommand(turn(0));
  }

  /**
   * @return new pivot (sim if nothing)
   */
  public static EndEffectorPivot create() {
    return new EndEffectorPivot(Robot.isReal() ? new RealEndEffector() : new SimEndEffector());
  }

  /**
   * @return no end effector
   */
  public static EndEffectorPivot none() {
    return new EndEffectorPivot(new NoEndEffector());
  }

  /**
   * @param voltage
   * @return set voltage of end effector
   */
  public Command turn(double voltage) {
    return runOnce(() -> hardware.setVoltage(voltage));
  }

  /**
   * @return stop it from turning
   */
  public Command stop() {
    return turn(0);
  }

  /**
   * @param angle
   * @param pid set up pid
   */
  public void update(double angle, ProfiledPIDController pid) {
    double rads = MathUtil.clamp(angle, MIN_ANGLE, MAX_ANGLE);
    double pidVoltage = pid.calculate(hardware.position(), rads);
    double ffVoltage = ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);
    hardware.setVoltage(pidVoltage + ffVoltage);
  }

  /**
   * @param angle pid with just the angle
   */
  public void update(double angle) {
    update(angle, pid);
  }

  /**
   * @param angle
   * @return go to a specific angle
   */
  public Command goTo(double angle) {
    return run(() -> update(angle)).withName("go to angle");
  }

  /**
   * @param angle
   * @return go to an angle that is input as double supplier
   */
  public Command goTo(DoubleSupplier angle) {
    return run(() -> update(angle.getAsDouble())); // .withName("go to angle"));
  }

  /**
   * @return automate scoring
   */
  public Command score() {
    return run(() -> goTo(SCORE_ANGLE));
  }

  /**
   * @return the position
   */
  @Logged
  public double position() {
    return hardware.position();
  }

  /**
   * @return the velocity
   */
  @Logged
  public double velocity() {
    return hardware.velocity();
  }

  /**
   * @return where we want the position to be
   */
  @Logged
  public double positionSetpoint() {
    return pid.getSetpoint().position;
  }

  /**
   * @return where we want the velocity to be
   */
  @Logged
  public double velocitySetpoint() {
    return pid.getSetpoint().velocity;
  }

  /**
   * @param angle
   * @return is the angle within a sufficient tolerance?
   */
  public boolean atPosition(double angle) {
    return Math.abs(angle - position()) < POSITION_TOLERANCE.in(Radians);
  }

  /**
   * @return is the pivot where we want it to be?
   */
  @Logged
  public boolean atGoal() {
    return pid.atGoal();
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
    LoggingUtils.log(
        "/Robot/Pivot/command",
        Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
