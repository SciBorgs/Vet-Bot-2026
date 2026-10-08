package org.sciborgs1155.robot.endEffectorHold;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

public class EndEffectorHoldConstants {
  public static final Current CURRENT_LIMIT = Amps.of(30);

  public static final double GEARING = 5;

  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;

  public static final double MAX_VELOCITY = 20;
  public static final double MAX_ACCEL = 20;

  public static final Angle POSITION_TOLERANCE = Radians.of(10);
}
