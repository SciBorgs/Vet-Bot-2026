package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimPivot implements PivotIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          GEARBOX,
          GEARING,
          MOI,
          LENGTH.in(Meters),
          MIN_ANGLE.in(Radians),
          MAX_ANGLE.in(Radians),
          true,
          START_ANGLE.in(Radians));

  @Override
  public void setVoltage(double voltage) {
    sim.setInputVoltage(voltage);
    sim.update(PERIOD.in(Seconds));
  }

  // get the position of the slapdown
  @Override
  public double position() {
    return sim.getAngleRads();
  }

  @Override
  public void close() throws Exception {
    sim.setInputVoltage(0);
  }

  @Override
  public double current() {
    return sim.getCurrentDrawAmps();
  }
}
