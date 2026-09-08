package org.sciborgs1155.robot.endEffector;

public interface EndEffectorIO extends AutoCloseable {
  void setVoltage(double voltage);

  double position();

  double velocity();
}
