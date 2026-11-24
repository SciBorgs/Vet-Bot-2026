package org.sciborgs1155.robot.endeffectorpivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public final class EndEffectorPivotConstants {
  public static final Current CURRENT_LIMIT = Amps.of(30);

  public static final double GEARING = 5;

  @SuppressWarnings("PMD")
  public static final double kP = 5.0; // change

  @SuppressWarnings("PMD")
  public static final double kI = 0.0; // change

  @SuppressWarnings("PMD")
  public static final double kD = 0.1; // change

  @SuppressWarnings("PMD")
  public static final double kS = 0.3; // change

  @SuppressWarnings("PMD")
  public static final double kG = 0.9; // change

  @SuppressWarnings("PMD")
  public static final double kV = 0.2; // change

  @SuppressWarnings("PMD")
  public static final double kA = 0.6; // change

  public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(10);
  public static final double MAX_ACCEL = 20;

  public static final Angle POSITION_TOLERANCE = Radians.of(0.1);

  public static final Velocity<VoltageUnit> RAMP_RATE = Volts.of(0.3).per(Second);
  public static final Voltage STEP_VOLTAGE = Volts.of(0.7);
  public static final Time TIME_OUT = Seconds.of(30);

  public static final Boolean TUNING = true;

  public static final double START_ANGLE = 0;

  public static final double MAX_ANGLE = 2 * Math.PI;
  public static final double MIN_ANGLE = 0;

  public static final double SCORE_ANGLE = 0;

  public static final DCMotor GEARBOX = DCMotor.getNeo550(1);
  public static final double MOI = 0.2135396026; // moment of inertia, change
  public static final Distance LENGTH = Inches.of(18.5); // change

  private EndEffectorPivotConstants() {}
}
