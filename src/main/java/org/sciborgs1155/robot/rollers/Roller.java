package org.sciborgs1155.robot.rollers;

import org.sciborgs1155.lib.SimpleMotor;
import static org.sciborgs1155.robot.Ports.Roller.*;
import org.sciborgs1155.robot.Robot;
import static org.sciborgs1155.robot.rollers.RollerConstants.CURRENT_LIMIT;
import static org.sciborgs1155.robot.rollers.RollerConstants.GEARING;
import static org.sciborgs1155.robot.rollers.RollerConstants.ROLLER_POWER;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Amps;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Roller extends SubsystemBase implements AutoCloseable {
    private final SimpleMotor left;
    private final SimpleMotor right;


    public Roller(SimpleMotor left, SimpleMotor right) {
        this.left = left;
        this.right = right;
        setDefaultCommand(stop());
    }

    public static Roller create() {
        return Robot.isReal() ? new Roller(realMotor(LEFT_MOTOR), realMotor(RIGHT_MOTOR)) : none();
    }

    private static SimpleMotor realMotor(int port) {
        final TalonFX motor = new TalonFX(port);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = GEARING;
        return SimpleMotor.talon(motor, config);
    }

    public static Roller none() {
        return new Roller(SimpleMotor.none(), SimpleMotor.none());
    }

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