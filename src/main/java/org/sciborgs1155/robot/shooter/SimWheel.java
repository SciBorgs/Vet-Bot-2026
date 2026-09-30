package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.shooter.ShooterConstants.GEARING;
import static org.sciborgs1155.robot.shooter.ShooterConstants.MOI;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Flywheel simulation using two Kraken X60 motors and the reference robot's inertia. */
public final class SimWheel implements WheelIO {
  private final FlywheelSim flywheel =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(2), MOI, GEARING),
          DCMotor.getKrakenX60(2));

  @Override
  public void setVoltage(double volts) {
    flywheel.setInputVoltage(volts);
    flywheel.update(PERIOD.in(Seconds));
  }

  @Override
  public double velocity() {
    return flywheel.getAngularVelocityRadPerSec();
  }

  @Override
  public void close() {}
}
