package org.sciborgs1155.robot.endEffectorPivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Ports.EndEffector.ENCODER;
import static org.sciborgs1155.robot.Ports.EndEffector.MOTOR_PIVOT;
import static org.sciborgs1155.robot.endEffectorPivot.EndEffectorPivotConstants.CURRENT_LIMIT;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.sciborgs1155.lib.FaultLogger;

public class RealEndEffector implements EndEffectorIO {
  private final SparkMax motor;
  private final CANcoder encoder;

  public RealEndEffector() {
    motor = new SparkMax(MOTOR_PIVOT, MotorType.kBrushless);
    encoder = new CANcoder(ENCODER);

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    motorConfig.smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.inverted(true);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    FaultLogger.register(motor);
    FaultLogger.register(encoder);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public double position() {
    return encoder.getAbsolutePosition().getValue().in(Radians);
  }

  @Override
  public double velocity() {
    return encoder.getVelocity().getValue().in(RadiansPerSecond);
  }

  @Override
  public double current() {
    return motor.getOutputCurrent();
  }

  @Override
  public void resetPosition() {
    encoder.setPosition(0);
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }
}
