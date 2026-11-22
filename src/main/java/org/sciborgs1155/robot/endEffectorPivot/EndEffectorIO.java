package org.sciborgs1155.robot.endeffectorpivot;

public interface EndEffectorIO extends AutoCloseable {
  /**
   * @param voltage set the voltage
   */
  void setVoltage(double voltage);

  /**
   * @return the position of the pivot
   */
  double position();

  /**
   * @return the velocity
   */
  double velocity();

  /**
   * @return the current
   */
  double current();

  /** reset the encoder */
  void resetPosition();
}
