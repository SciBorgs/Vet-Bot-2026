package org.sciborgs1155.robot.endEffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class EndEffectorConstants {
  public static final double kP = 0;
  public static final double kI = 0;
  public static final double kD = 0;

  public static final double MAX_VELOCITY = 10;
  public static final double MAX_ACCEL = 10;

  public static final double kS = 0;
  public static final double kV = 0;
  public static final double kG = 0;
  public static final double kA = 0;

  public static final Angle POSITION_TOLERANCE = Radians.of(0.5);

  public static final Angle MAX_ANGLE = Radians.of(0.8);
  public static final Angle MIN_ANGLE = Radians.of(0);

  public static final Velocity<VoltageUnit> RAMP_RATE = Volts.of(0.3).per(Second);
  public static final Voltage STEP_VOLTS = Volts.of(0.7);
  public static final Time TIME_OUT = Seconds.of(30);

  public static final Current CURRENT_LIMIT = Amps.of(20);

  public static final DCMotor GEARBOX = DCMotor.getKrakenX60(1);
  public static final double GEARING = 5; // TODO: get real value
  public static final double MOI = 0.1; // TODO: get real value
  public static final Distance LENGTH = Inches.of(5);

  public static final Time PERIOD = Seconds.of(3);
}
