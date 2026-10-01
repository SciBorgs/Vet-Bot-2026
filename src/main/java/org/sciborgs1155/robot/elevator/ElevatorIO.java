package org.sciborgs1155.robot.elevator;

public interface ElevatorIO extends AutoCloseable {
  void setVoltage(double volts);

  void resetPosition();

  double position();

  double velocity();
}
