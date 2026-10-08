package org.sciborgs1155.robot.endEffectorHold;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Ports.EndEffector.*;
import static org.sciborgs1155.robot.endEffectorHold.EndEffectorHoldConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
      new ProfiledPIDController(kP, kI, kP, new Constraints(MAX_VELOCITY, MAX_ACCEL));
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);

  public EndEffectorHold(SimpleMotor hardware) {
    this.hardware = hardware;
    pid.setTolerance(POSITION_TOLERANCE.in(Radians));
    setDefaultCommand(stop());
  }

  public static EndEffectorHold create() {
    return Robot.isReal() ? new EndEffectorHold(realMotor()) : none();
  }

  private static SimpleMotor realMotor() {
    final TalonFX motor = new TalonFX(MOTOR_PIVOT);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = GEARING;
    return SimpleMotor.talon(motor, config);
  }

  public static EndEffectorHold none() {
    return new EndEffectorHold(SimpleMotor.none());
  }

  public Command in(double voltage) {
    return run(() -> hardware.set(voltage));
  }

  public Command out(double voltage) {
    return run(() -> hardware.set(-voltage));
  }

  public Command stop() {
    return in(0);
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
