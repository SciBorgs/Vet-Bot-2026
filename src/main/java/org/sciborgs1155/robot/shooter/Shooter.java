package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.Constants.TUNING;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.Control.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Robot;

public final class Shooter extends SubsystemBase implements AutoCloseable {
  private final WheelIO hardware;

  @Logged
  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          P,
          I,
          D,
          new TrapezoidProfile.Constraints(MAX_PROFILE_VELOCITY, MAX_PROFILE_ACCELERATION));
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(S, V, A, PERIOD.in(Seconds));
  private final SysIdRoutine characterization;

  @NotLogged private final DoubleEntry tuningP = Tuning.entry("Robot/tuning/shooter/K_P", P);
  @NotLogged private final DoubleEntry tuningI = Tuning.entry("Robot/tuning/shooter/K_I", I);
  @NotLogged private final DoubleEntry tuningD = Tuning.entry("Robot/tuning/shooter/K_D", D);
  @NotLogged private final DoubleEntry tuningS = Tuning.entry("Robot/tuning/shooter/S", S);
  @NotLogged private final DoubleEntry tuningV = Tuning.entry("Robot/tuning/shooter/V", V);
  @NotLogged private final DoubleEntry tuningA = Tuning.entry("Robot/tuning/shooter/A", A);

  /**
   * Returns the shooter subsystem.
   *
   * @return Creates the real or simulated shooter based on {@link Robot#isReal()}.
   */
  public static Shooter create() {
    return Robot.isReal() ? new Shooter(new RealWheel()) : new Shooter(new SimWheel());
  }

  /**
   * Returns a shooter subsystem with no hardware.
   *
   * @return A shooter that does not drive any real motors.
   */
  public static Shooter none() {
    return new Shooter(new FakeWheel());
  }

  /**
   * Sets the shooter's default command and PID tolerance.
   *
   * @param hardware Takes in the WheelIO class.
   */
  public Shooter(WheelIO hardware) {
    this.hardware = hardware;
    controller.setTolerance(VELOCITY_TOLERANCE.in(RadiansPerSecond));

    characterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(1),
                Volts.of(10),
                Seconds.of(11),
                state -> SignalLogger.writeString("shooter state", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> hardware.setVoltage(voltage.in(Volts)), null, this, "shooter"));
    SmartDashboard.putData(
        "Robot/shooter/quasistatic reverse", characterization.quasistatic(Direction.kReverse));
    SmartDashboard.putData(
        "Robot/shooter/quasistatic forward", characterization.quasistatic(Direction.kForward));
    SmartDashboard.putData(
        "Robot/shooter/dynamic reverse", characterization.dynamic(Direction.kReverse));
    SmartDashboard.putData(
        "Robot/shooter/dynamic forward", characterization.dynamic(Direction.kForward));

    setDefaultCommand(stopShooter());
  }

  /**
   * Updates the velocity setpoint of the motor.
   *
   * @param velocitySetpoint The value of the velocity setpoint (rad/s).
   * @param noDeceleration If true, sends no negative voltage so the flywheel can coast down rather
   *     than fighting itself.
   */
  public void update(double velocitySetpoint, boolean noDeceleration) {
    double goal =
        MathUtil.clamp(
            velocitySetpoint,
            -MAX_VELOCITY.in(RadiansPerSecond),
            MAX_VELOCITY.in(RadiansPerSecond));
    double previousSetpoint = controller.getSetpoint().position;
    double feedbackVolts = controller.calculate(velocity(), goal);
    double feedforwardVolts =
        feedforward.calculateWithVelocities(previousSetpoint, controller.getSetpoint().position);
    double volts = MathUtil.clamp(feedbackVolts + feedforwardVolts, -MAX_VOLTAGE, MAX_VOLTAGE);
    hardware.setVoltage(noDeceleration ? Math.max(0, volts) : volts);
  }

  /**
   * Updates the velocity setpoint of the motor.
   *
   * @param velocitySetpoint The value of the velocity setpoint (rad/s).
   */
  public void update(double velocitySetpoint) {
    update(velocitySetpoint, false);
  }

  /**
   * Checks if the PID controller is at the velocity setpoint.
   *
   * @return Whether the controller is at the current setpoint.
   */
  @Logged
  public boolean atSetpoint() {
    return controller.atSetpoint();
  }

  /**
   * Checks if the current velocity matches the target velocity within tolerance.
   *
   * @param desiredVelocity The target velocity to compare against.
   * @return True if the actual wheel speed is within tolerance.
   */
  public boolean atVelocity(double desiredVelocity) {
    return Math.abs(desiredVelocity - velocity()) < VELOCITY_TOLERANCE.in(RadiansPerSecond);
  }

  /**
   * Returns the controller setpoint.
   *
   * @return The current target velocity for the shooter.
   */
  @Logged
  public double setpoint() {
    return controller.getSetpoint().position;
  }

  /**
   * Returns the shooter velocity.
   *
   * @return The current flywheel velocity in radians per second.
   */
  @Logged
  public double velocity() {
    return hardware.velocity();
  }

  /**
   * Runs the shooter at a specified velocity.
   *
   * @param velocity The desired velocity as a DoubleSupplier.
   * @return The command to set the shooter's velocity.
   */
  public Command runShooter(DoubleSupplier velocity) {
    return run(() -> update(velocity.getAsDouble()))
        .finallyDo(() -> hardware.setVoltage(0))
        .withName("running shooter");
  }

  /**
   * Runs the shooter at a specified velocity.
   *
   * @param velocity The desired velocity as a double.
   * @return The command to set the shooter's velocity.
   */
  public Command runShooter(double velocity) {
    return runShooter(() -> velocity);
  }

  /**
   * Idles the shooter while letting the flywheel coast down naturally.
   *
   * @return The command that keeps the flywheel near the idle speed.
   */
  public Command idleShooter() {
    return run(() -> update(IDLE_VELOCITY.in(RadiansPerSecond), true))
        .finallyDo(() -> hardware.setVoltage(0))
        .withName("idle shooter");
  }

  /**
   * Stops the shooter and clears the motor output.
   *
   * @return The command used as the subsystem's default stop behavior.
   */
  public Command stopShooter() {
    return run(
            () -> {
              hardware.setVoltage(0);
              controller.reset(velocity());
            })
        .withName("stop shooter");
  }

  /**
   * Manual control of the shooter with a controller input stream.
   *
   * @param input The controller value to use for manual control.
   */
  public Command manualShooter(InputStream input) {
    return runShooter(
            input
                .deadband(0.15, 1)
                .scale(MAX_VELOCITY.in(RadiansPerSecond))
                .scale(PERIOD.in(Seconds))
                .add(this::setpoint))
        .withName("manual shooter");
  }

  /**
   * Does a quick check to confirm the shooter can reach a target speed.
   *
   * @return A command that validates the flywheel speed against a fixed goal.
   */
  public Command systemsCheck() {
    double goal = 200;
    return runShooter(goal)
        .until(this::atSetpoint)
        .withTimeout(5)
        .andThen(
            FaultLogger.reportTrue(
                () -> atVelocity(goal),
                "Shooter speed",
                () -> "expected: " + goal + "; actual: " + velocity()));
  }

  @Override
  public void periodic() {
    Command command = getCurrentCommand();
    LoggingUtils.log("Robot/shooter/current command", command == null ? "None" : command.getName());
    LoggingUtils.log("Robot/shooter/velocity", velocity());
    if (TUNING) {
      controller.setP(tuningP.get());
      controller.setI(tuningI.get());
      controller.setD(tuningD.get());
      feedforward.setKs(tuningS.get());
      feedforward.setKv(tuningV.get());
      feedforward.setKa(tuningA.get());
    }
  }

  @Override
  public void close() {
    hardware.setVoltage(0);
    hardware.close();
  }
}
