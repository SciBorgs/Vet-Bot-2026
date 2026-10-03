package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class SimElevator implements ElevatorIO {
  private final ElevatorSim sim;

  public SimElevator() {
    sim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                DCMotor.getNeoVortex(2), MASS.in(Kilogram), RADIUS.in(Meters), GEARING),
            DCMotor.getNeoVortex(2),
            MIN_HEIGHT.in(Meters),
            MAX_HEIGHT.in(Meters),
            true,
            MIN_HEIGHT.in(Meters));
  }

  @Override
  public void setVoltage(double volts) {
    sim.setInput(volts);
    sim.update(PERIOD.in(Seconds));
  }

  @Override
  public double position() {
    return sim.getPositionMeters();
  }

  @Override
  public void resetPosition() {
    sim.setState(0, 0);
  }

  @Override
  public double velocity() {
    return sim.getVelocityMetersPerSecond();
  }

  @Override
  public void close() throws Exception {}
}
