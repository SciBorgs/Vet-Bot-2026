package org.sciborgs1155.robot.endEffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Ports.EndEffector.*;
import static org.sciborgs1155.robot.endEffector.EndEffectorConstants.CURRENT_LIMIT;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

public class RealEndEffector implements EndEffectorIO {
  private final TalonFX motor;
  private final CANcoder encoder;

  public RealEndEffector() {
    motor = new TalonFX(MOTOR_CLAW);
    encoder = new CANcoder(ENCODER);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    motorConfig.Feedback.FeedbackRemoteSensorID = ENCODER;
    motorConfig.Feedback.SensorToMechanismRatio = 1;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    encoderConfig.MagnetSensor.MagnetOffset = 0;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0;

    motor.getConfigurator().apply(motorConfig);
    encoder.getConfigurator().apply(encoderConfig);

    TalonUtils.addMotor(motor);
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
    return encoder.getVelocity().getValueAsDouble();
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }
}
