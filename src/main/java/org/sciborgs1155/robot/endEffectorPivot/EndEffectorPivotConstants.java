package org.sciborgs1155.robot.endEffectorPivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public class EndEffectorPivotConstants {
  public static final Current CURRENT_LIMIT = Amps.of(30);

  public static final double GEARING = 5;

  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kS = 0.0;
  public static final double kG = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;

  public static final double MAX_VELOCITY = 20;
  public static final double MAX_ACCEL = 20;

  public static final Angle POSITION_TOLERANCE = Radians.of(10);

  public static final double START_ANGLE = 0;

  public static final double MAX_ANGLE = 0;
  public static final double MIN_ANGLE = 0;

  public static final double SCORE_ANGLE = 0;

  public static final DCMotor GEARBOX = DCMotor.getNeo550(1);
  public static final double MOI = 0.2135396026; // moment of inertia, change
  public static final Distance LENGTH = Inches.of(18.5); // change
}
