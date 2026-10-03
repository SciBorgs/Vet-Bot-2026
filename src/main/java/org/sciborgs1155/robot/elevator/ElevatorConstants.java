package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

public class ElevatorConstants {

  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kS = 0.0;
  public static final double kG = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;

  public static final Mass MASS = Kilogram.of(0);
  public static final Distance RADIUS = Meters.of(0);
  public static final double GEARING = 0;

  public static final Distance MIN_HEIGHT = Meters.of(0);
  public static final Distance MAX_HEIGHT = Meters.of(0);

  public static final double CONVERSION_FACTOR = GEARING * 2 * Math.PI * RADIUS.in(Meters);

  public static final Current CURRENT_LIMIT = Amps.of(60);

  public static final double HOMING_OUTPUT = -0.1;

  public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(0);
  public static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(0);

  public static final double HOMING_VOLTAGE = 0;
  public static final double VELOCITY_TOLERANCE = 0;
}
