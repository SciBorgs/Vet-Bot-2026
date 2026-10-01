package org.sciborgs1155.robot.shooter;

/** Hardware boundary for the shooter flywheel. */
public interface WheelIO extends AutoCloseable {
  /**
   * Sets the flywheel motor voltage.
   *
   * @param volts The requested motor voltage.
   */
  void setVoltage(double volts);

  /**
   * Returns the flywheel velocity.
   *
   * @return The flywheel speed in radians per second.
   */
  double velocity();

  /** Closes the hardware interface and releases any resources. */
  @Override
  void close();
}
