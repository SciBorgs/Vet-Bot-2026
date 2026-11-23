package org.sciborgs1155.robot.endeffectorpivot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.endeffectorpivot.EndEffectorPivotConstants.*;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimEndEffector implements EndEffectorIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          GEARBOX, GEARING, MOI, LENGTH.in(Inches), MIN_ANGLE, MAX_ANGLE, true, START_ANGLE);

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
  public double current() {
    return sim.getCurrentDrawAmps();
  }

  @Override
  public void resetPosition() {
    sim.setState(MIN_ANGLE, 0);
  }

  @Override
  public void close() throws Exception {
    sim.setInputVoltage(0);
  }
}
