package org.sciborgs1155.robot.endEffectorPivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Ports.EndEffector.*;
import static org.sciborgs1155.robot.endEffector.EndEffectorConstants.CURRENT_LIMIT;
import static org.sciborgs1155.robot.endEffector.EndEffectorConstants.GEARING;
import static org.sciborgs1155.robot.endEffector.EndEffectorConstants.MAX_ACCEL;
import static org.sciborgs1155.robot.endEffector.EndEffectorConstants.MAX_VELOCITY;
import static org.sciborgs1155.robot.endEffector.EndEffectorConstants.POSITION_TOLERANCE;
import static org.sciborgs1155.robot.endEffector.EndEffectorConstants.kD;
import static org.sciborgs1155.robot.endEffector.EndEffectorConstants.kI;
import static org.sciborgs1155.robot.endEffector.EndEffectorConstants.kP;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.lib.SimpleMotor;
import org.sciborgs1155.robot.Robot;

public class EndEffectorPivot extends SubsystemBase implements AutoCloseable {
  private final SimpleMotor hardware;
  private final ProfiledPIDController pid = new ProfiledPIDController(kP, kI, kP, new Constraints(MAX_VELOCITY, MAX_ACCEL));
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);

  public EndEffectorPivot(SimpleMotor hardware) {
    this.hardware = hardware;
    pid.setTolerance(POSITION_TOLERANCE.in(Radians));
    setDefaultCommand(stop());
  }

  public static EndEffectorPivot create() {
    return Robot.isReal() ? new EndEffectorPivot(realMotor()) : none();
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

  public static EndEffectorPivot none() {
    return new EndEffectorPivot(SimpleMotor.none());
  }

  public Command turn(double voltage) {
    return run(() -> hardware.set(voltage));
  }

  public Command stop() {
    return turn(0);
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
