package org.sciborgs1155.robot;

import java.util.Random;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Radians;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.sciborgs1155.robot.pivot.Pivot;
import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;
import static org.sciborgs1155.lib.UnitTestingUtil.fastForward;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;

public class PivotTest {
    private Pivot pivot; 
    private final Random random = new Random();

    @BeforeEach
    private void setup() {
        pivot = pivot.create();
        setupTests();
    }

    @AfterEach
    public void destroy() throws Exception {
        reset(pivot);
    }

    @Test
    public void orientation() {
        double setpoint = random.nextDouble(MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians));
        CommandScheduler.getInstance()
            .schedule(pivot.goTo(setpoint).withDeadline(Commands.waitSeconds(7)));
        fastForward(Seconds.of(6));
        assertEquals(setpoint, pivot.position(), POSITION_TOLERANCE.in(Radians));
    }
}
