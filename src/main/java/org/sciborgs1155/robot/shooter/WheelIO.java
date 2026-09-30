package org.sciborgs1155.robot.shooter;

/** Hardware boundary for the shooter flywheel. Velocity is measured in radians per second. */
public interface WheelIO extends AutoCloseable {
  void setVoltage(double volts);

  double velocity();

  @Override
  void close();
}
