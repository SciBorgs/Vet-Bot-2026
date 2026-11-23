package org.sciborgs1155.robot.endeffectorpivot;

public class NoEndEffector implements EndEffectorIO {

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public double position() {
    return 0;
  }

  @Override
  public double velocity() {
    return 0;
  }

  @Override
  public double current() {
    return 0;
  }

  @Override
  public void resetPosition() {}

  @Override
  public void close() throws Exception {}
}
