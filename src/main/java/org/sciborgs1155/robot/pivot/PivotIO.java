package org.sciborgs1155.robot.pivot;

public interface PivotIO extends AutoCloseable {
  /**
   * @param voltage of the intake extending (which is an arm)
   */
  void setVoltage(double voltage);

  /**
   * @return the position of the intake when it is extended
   */
  double position();

  /**
   * @return the current to the motor
   */
  double current();
}
