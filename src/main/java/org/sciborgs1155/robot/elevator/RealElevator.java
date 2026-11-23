package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Elevator.*;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class RealElevator implements ElevatorIO {

  private final SparkMax leader = new SparkMax(FRONT_LEADER, MotorType.kBrushless);
  private final SparkMax follower = new SparkMax(BACK_FOLLOWER, MotorType.kBrushless);

  /** creates elevator w/ real motor configs */
  public RealElevator() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit((int) Math.round(CURRENT_LIMIT.in(Amps)));

    config.encoder.positionConversionFactor(1.0 / CONVERSION_FACTOR);
    config.encoder.velocityConversionFactor(1.0 / CONVERSION_FACTOR / 60);

    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.apply(config);

    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.apply(config);
    followerConfig.follow(leader);

    leader.getEncoder().setPosition(0);
    follower.getEncoder().setPosition(0);

    leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    follower.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public double position() {
    return leader.getEncoder().getPosition();
  }

  @Override
  public double velocity() {
    return leader.getEncoder().getVelocity();
  }

  @Override
  public void resetPosition() {
    leader.getEncoder().setPosition(0);
  }

  @Override
  public void close() throws Exception {
    leader.close();
    follower.close();
  }
}
