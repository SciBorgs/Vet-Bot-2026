package org.sciborgs1155.robot.endeffectorpivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Ports.EndEffector.ENCODER;
import static org.sciborgs1155.robot.Ports.EndEffector.MOTOR_PIVOT;
import static org.sciborgs1155.robot.endeffectorpivot.EndEffectorPivotConstants.CURRENT_LIMIT;

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

  /** instantiates the hardware */
  public RealEndEffector() {
    motor = new SparkMax(MOTOR_PIVOT, MotorType.kBrushless);
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

  /** sets the voltage */
  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /** where is the pivot */
  @Override
  public double position() {
    return encoder.getAbsolutePosition().getValue().in(Radians);
  }

  /** what is the velocity */
  @Override
  public double velocity() {
    return encoder.getVelocity().getValue().in(RadiansPerSecond);
  }

  /** what is the current */
  @Override
  public double current() {
    return motor.getOutputCurrent();
  }

  /** reset the encoder */
  @Override
  public void resetPosition() {
    encoder.setPosition(0);
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }
}
