package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.Ports.Pivot.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.sciborgs1155.lib.FaultLogger;

public class RealPivot implements PivotIO {
  private final SparkMax motor;
  private final CANcoder encoder;

  /** Returns a new RealPivot object. */
  public RealPivot() {
    motor = new SparkMax(MOTOR, MotorType.kBrushless);
    encoder = new CANcoder(ENCODER);

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    // CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

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
  public double current() {
    return motor.getOutputCurrent();
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }
}
