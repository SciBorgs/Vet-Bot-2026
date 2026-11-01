package org.sciborgs1155.robot.endEffectorPivot;

public class NoEndEffector implements EndEffectorIO {

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public double position() {
    return position();
  }

  @Override
  public double velocity() {
    return velocity();
  }

  @Override
  public double current() {
    return current();
  }

  @Override
  public void resetPosition() {}

  @Override
  public void close() throws Exception {}
}
