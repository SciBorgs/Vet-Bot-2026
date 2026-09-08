package org.sciborgs1155.robot.endEffector;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.endEffector.EndEffectorConstants.*;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimEndEffector implements EndEffectorIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          GEARBOX,
          GEARING,
          MOI,
          LENGTH.in(Meters),
          MIN_ANGLE.in(Radians),
          MAX_ANGLE.in(Radians),
          true,
          MAX_ANGLE.in(Radians),
          0.01);

  @Override
  public void setVoltage(double voltage) {
    sim.setInputVoltage(voltage);
    sim.update(PERIOD.in(Seconds));
  }

  @Override
  public double position() {
    return sim.getAngleRads();
  }

  @Override
  public double velocity() {
    return sim.getVelocityRadPerSec();
  }

  @Override
  public void close() throws Exception {
    sim.setInputVoltage(0);
  }
}
