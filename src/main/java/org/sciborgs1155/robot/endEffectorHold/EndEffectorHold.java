package org.sciborgs1155.robot.endeffectorhold;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Ports.EndEffector.*;
import static org.sciborgs1155.robot.endeffectorhold.EndEffectorHoldConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.lib.SimpleMotor;
import org.sciborgs1155.robot.Robot;

public class EndEffectorHold extends SubsystemBase implements AutoCloseable {
  private final SimpleMotor hardware;
  private final ProfiledPIDController pid =
      new ProfiledPIDController(kP, kI, kD, new Constraints(MAX_VELOCITY, MAX_ACCEL));
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);

  /** Instantiates the hardware */
  public EndEffectorHold(SimpleMotor hardware) {
    this.hardware = hardware;
    pid.setTolerance(POSITION_TOLERANCE.in(Radians));
    setDefaultCommand(stop());
  }

  /** creates a new hold if the robot is real */
  public static EndEffectorHold create() {
    return Robot.isReal() ? new EndEffectorHold(realMotor()) : none();
  }

  private static SimpleMotor realMotor() {
    final MotorType motorType = MotorType.kBrushless;
    final SparkMax motor = new SparkMax(MOTOR_CLAW, motorType);
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));
    config.idleMode(IdleMode.kBrake);
    config.inverted(true);
    // config.Feedback.SensorToMechanismRatio = GEARING;
    // TODO: Config gearing on encoder
    return SimpleMotor.spark(motor, config);
  }

  /** makes no end effector */
  public static EndEffectorHold none() {
    return new EndEffectorHold(SimpleMotor.none());
  }

  /** intakes a ball */
  public Command in() {
    return run(() -> hardware.setVoltage(INTAKING_VOLTAGE));
  }

  /** outtakes */
  public Command out() {
    return run(() -> hardware.setVoltage(-INTAKING_VOLTAGE));
  }

  /**
   * @param volts how hard to spin the ball out (sign is ignored)
   * @return outtakes at the given voltage
   */
  public Command out(double volts) {
    return run(() -> hardware.setVoltage(-Math.abs(volts)));
  }

  /** stops it from turning */
  public Command stop() {
    return run(() -> hardware.set(0));
  }

  /** closes it */
  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
