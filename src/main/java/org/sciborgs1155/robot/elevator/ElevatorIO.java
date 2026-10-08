package org.sciborgs1155.robot.elevator;

public interface ElevatorIO extends AutoCloseable {

  /** sets voltage to volts */
  void setVoltage(double volts);

  /** retracts to reset position */
  void resetPosition();

  /** returns the current position */
  double position();

  /** returns the current velocity */
  double velocity();
}
