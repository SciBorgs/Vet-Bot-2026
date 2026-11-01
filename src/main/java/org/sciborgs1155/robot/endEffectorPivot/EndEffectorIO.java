package org.sciborgs1155.robot.endEffectorPivot;

public interface EndEffectorIO extends AutoCloseable {
  void setVoltage(double voltage);

  double position();

  double velocity();

  double current();

  void resetPosition();
}
