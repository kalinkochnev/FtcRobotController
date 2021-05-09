package org.firstinspires.ftc.teamcode.helpers;

import fakes.*;
import fakes.drive.FakeDcMotor;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

public class TestMotors {
    private FakeHardwareMap hardwareMap;
    private String[] motorNames = {"lfMotor", "rfMotor", "rbMotor", "lbMotor"};
    private FakeDcMotor[] fakeDcMotors = new FakeDcMotor[4];
    private Motors motors;

    @BeforeEach
    public void setUp() {
        hardwareMap = new FakeHardwareMap();

        for (int i = 0; i < 4; i++) {
            fakeDcMotors[i] = new FakeDcMotor();
            hardwareMap.addDevice(motorNames[i], fakeDcMotors[i]);
        }

        FakeTelemetry telemetry = new FakeTelemetry();
        motors = new Motors(hardwareMap);
    }

    @Test
    public void testMotorTranslationWeights() {
        // At 0 degrees
        double[] weights = motors.translationWeights(0, 1);
        double[] expected = {1/Math.sqrt(2), -1/Math.sqrt(2), 1/Math.sqrt(2), -1/Math.sqrt(2)}; // lf, rf, rb, lb
        assertArrayEquals(weights, expected);
    }

    @Test
    public void testCompensateDistance() {
        // Lets say the robot travels 0.98 meters horizontally and 0.90 meters verically when it should be doing 1 and 1
        motors.realX = 0.98;
        motors.realY = 0.9;
        motors.expectedDist = Math.sqrt(2);

        // We expect the x component to be 1.02 bigger and the y to be 1.1 bigger
        double xComp = 7;
        double yComp = 32;
        double distanceToGo = motors.compensateDistance(Math.sqrt(Math.pow(xComp, 2) + Math.pow(yComp, 2)), Math.atan(yComp/xComp));

        double newHypotenuse = Math.sqrt(Math.pow(xComp * (1 / motors.realX), 2) + Math.pow(yComp * 1/(motors.realY), 2));

        assertEquals(newHypotenuse, distanceToGo, 0.00001);

    }

    @Test
    public void testAddEncoderVectors() {
//        //  Moving 45 degrees
//        int[] fakeTicks = new int[]{100, 100, 100, 100};
//        PVector resultantTicks = this.motors.getNetEncoderVector(fakeTicks);
//        assertEquals(new PVector(0, 100), resultantTicks);
//
//        // Wheels on opposite sides are moving together
//        fakeTicks = new int[]{100, -100, 100, -100};
//        resultantTicks = this.motors.getNetEncoderVector(fakeTicks);
//        assertEquals(new PVector(100, 0), resultantTicks);

        // Wheels are all moving in the same direction after motor directions are applied.
        int[] fakeTicks = new int[]{745, 710, 761, 742};
        int dist = Motors.distanceToEncoderTicks(0.6);
        PVector resultantTicks = this.motors.getNetEncoderVector(fakeTicks);
        double mag = resultantTicks.getMagnitude();
        assertEquals(new PVector(0, 100), resultantTicks);
    }

}
