package org.sciborgs1155.robot.shooter;

/** A no-op wheel for robots without shooter hardware. */
public final class FakeWheel implements WheelIO {
  @Override
  public void setVoltage(double volts) {}

  @Override
  public double velocity() {
    return 0.0;
  }

  @Override
  public void close() {}
}
