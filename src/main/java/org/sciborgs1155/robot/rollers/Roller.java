package org.sciborgs1155.robot.rollers;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Roller.*;
import static org.sciborgs1155.robot.rollers.RollerConstants.CURRENT_LIMIT;
// import static org.sciborgs1155.robot.rollers.RollerConstants.GEARING;
import static org.sciborgs1155.robot.rollers.RollerConstants.ROLLER_POWER;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.lib.SimpleMotor;
import org.sciborgs1155.robot.Robot;

public final class Roller extends SubsystemBase implements AutoCloseable {
  private final SimpleMotor left;
  private final SimpleMotor right;

  private Roller(SimpleMotor left, SimpleMotor right) {
    this.left = left;
    this.right = right;
    setDefaultCommand(stop());
  }
  /**
   * @return Returns a new roller object based on {@link Robot#isReal()}
   */
  public static Roller create() {
    return Robot.isReal() ? new Roller(realMotor(LEFT_MOTOR), realMotor(RIGHT_MOTOR)) : none();
  }

  /**
   * @return Returns a SimpleMotor using {@link SimpleMotor#spark(SparkMax motor, SparkMaxConfig config)}
   * 
   * @param port - the ID number of the port.
   */
  private static SimpleMotor realMotor(int port) {
    final SparkMax motor = new SparkMax(port, null);
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));
    config.idleMode(IdleMode.kBrake);
    config.inverted(true);
    // config.SensorToMechanismRatio = GEARING; TODO
    return SimpleMotor.spark(motor, config);
  }

  /**
   * @return Returns a roller object with no motors
   */
  public static Roller none() {
    return new Roller(SimpleMotor.none(), SimpleMotor.none());
  }

  /**
   * @return returns a command to 
   */
  public Command stop() {
    return spin(0);
  }

  /**
   * @return make the motors spin to intake the fuel
   */
  public Command intake() {
    return spin(ROLLER_POWER);
  }

  /**
   * @return make the motors spin outwards to outtake the fuel
   */
  public Command outtake() {
    return spin(-ROLLER_POWER);
  }

  public Command spin(double power) {
    return run(() -> left.set(power));
  }

  @Override
  public void close() throws Exception {
    left.close();
    right.close();
  }
}
