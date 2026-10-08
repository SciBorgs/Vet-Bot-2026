package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Ports.Pivot.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.sciborgs1155.lib.TalonUtils;

public class RealPivot implements PivotIO {
  private final TalonFX motor;
  private final CANcoder encoder;

  public RealPivot() {
    motor = new TalonFX(MOTOR);
    encoder = new CANcoder(ENCODER);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    motorConfig.Feedback.FeedbackRemoteSensorID = ENCODER;
    motorConfig.Feedback.SensorToMechanismRatio = 1;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // encoderConfig.MagnetSensor.MagnetOffset = -0.824462890625; tbd
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.9;

    motor.getConfigurator().apply(motorConfig);
    encoder.getConfigurator().apply(encoderConfig);

    TalonUtils.addMotor(motor);
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
  public double current() {
    return motor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }
}
