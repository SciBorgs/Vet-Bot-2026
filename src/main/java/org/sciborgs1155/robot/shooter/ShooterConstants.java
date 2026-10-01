package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

/** Shooter dimensions and initial control values from Rebuilt-2026. Tune on this robot. */
public final class ShooterConstants {
  public static final double GEARING = 1 / 0.8;
  public static final Distance RADIUS = Inches.of(2);
  public static final double MOI = 0.0015328465; // kg m^2
  public static final double MAX_VOLTAGE = 12.0;

  public static final AngularVelocity IDLE_VELOCITY = RadiansPerSecond.of(50);
  public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(400);
  public static final AngularVelocity VELOCITY_TOLERANCE = RadiansPerSecond.of(5);
  public static final AngularVelocity SYSTEMS_CHECK_VELOCITY = RadiansPerSecond.of(200);
  public static final double MAX_PROFILE_VELOCITY = 5000; // rad/s
  public static final double MAX_PROFILE_ACCELERATION = 2000; // rad/s^2

  public static final Current STATOR_CURRENT_LIMIT = Amps.of(100);
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(100);

  public static final class Control {
    public static final double P = 0.01;
    public static final double I = 0.0;
    public static final double D = 0.000003;
    public static final double S = 0.19071;
    public static final double V = 0.023602;
    public static final double A = 0.0024145;

    private Control() {}
  }

  private ShooterConstants() {}
}
