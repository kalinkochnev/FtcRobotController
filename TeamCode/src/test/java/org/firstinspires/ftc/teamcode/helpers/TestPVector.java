package org.firstinspires.ftc.teamcode.helpers;

import fakes.FakeHardwareMap;
import fakes.FakeTelemetry;
import fakes.drive.FakeDcMotor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

public class TestPVector {
    @Test
    public void testAdd() {
        PVector[] vectors = {new PVector(1, 1), new PVector(2, 2), new PVector(-1, -1), new PVector(-1, 1)};
        PVector result = PVector.add(vectors);
        assertEquals(1, result.x);
        assertEquals(3, result.y);
    }

}
