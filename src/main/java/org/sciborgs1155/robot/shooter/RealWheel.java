package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Constants.SHOOTING_CANIVORE;
import static org.sciborgs1155.robot.Ports.Shooter.FOLLOWER;
import static org.sciborgs1155.robot.Ports.Shooter.LEADER;
import static org.sciborgs1155.robot.shooter.ShooterConstants.GEARING;
import static org.sciborgs1155.robot.shooter.ShooterConstants.STATOR_CURRENT_LIMIT;
import static org.sciborgs1155.robot.shooter.ShooterConstants.SUPPLY_CURRENT_LIMIT;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

/** The two TalonFX shooter motors; the lower motor follows the upper motor. */
public final class RealWheel implements WheelIO {
  private final TalonFX leader = new TalonFX(LEADER, SHOOTING_CANIVORE);
  private final TalonFX follower = new TalonFX(FOLLOWER, SHOOTING_CANIVORE);

  public RealWheel() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = GEARING;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    follower.setControl(new Follower(LEADER, MotorAlignmentValue.Aligned));

    FaultLogger.register(leader);
    FaultLogger.register(follower);
    TalonUtils.addMotor(leader);
    TalonUtils.addMotor(follower);
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public double velocity() {
    return leader.getVelocity().getValue().in(RadiansPerSecond);
  }

  @Override
  public void close() {
    leader.close();
    follower.close();
  }
}
