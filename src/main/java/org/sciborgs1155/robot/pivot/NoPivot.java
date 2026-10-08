package org.sciborgs1155.robot.pivot;

public class NoPivot implements PivotIO {

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public double position() {
    return 0;
  }

  @Override
  public void close() throws Exception {}

  @Override
  public double current() {
    return 0;
  }
}
