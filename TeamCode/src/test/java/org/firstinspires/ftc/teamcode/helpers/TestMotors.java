package org.firstinspires.ftc.teamcode.helpers;

import fakes.FakeHardwareMap;
import fakes.FakeTelemetry;
import fakes.drive.FakeDcMotor;
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
    public void testAddEncoderVectors() {
        //  Moving 45 degrees
        int[] fakeTicks = new int[]{100, 100, 100, 100};
        PVector resultantTicks = this.motors.getNetPositionVector(fakeTicks);
        assertEquals(new PVector(0, 100).getMagnitude(), resultantTicks.getMagnitude());

        // Wheels on opposite sides are moving together, strafing sideways
        fakeTicks = new int[]{100, -100, 100, -100};
        resultantTicks = this.motors.getNetPositionVector(fakeTicks);
        assertEquals(new PVector(100, 0).getMagnitude(), resultantTicks.getMagnitude());

        // Wheels are all moving in the same direction after motor directions are applied.
        fakeTicks = new int[]{100, 100, 100, 100};
        resultantTicks = this.motors.getNetPositionVector(fakeTicks);
        assertEquals(new PVector(0, 100).getMagnitude(), resultantTicks.getMagnitude());
    }

    @Test
    public void testGetMotorIndexesForSide() {
        int[] left = {3, 0};
        assertArrayEquals(motors.getMotorsIndexesForSide(Motors.MotorSide.LEFT), left);

        int[] right = {1, 2};
        assertArrayEquals(motors.getMotorsIndexesForSide(Motors.MotorSide.RIGHT), right);

        int[] front = {0, 1};
        assertArrayEquals(motors.getMotorsIndexesForSide(Motors.MotorSide.FRONT), front);

        int[] back = {2, 3};
        assertArrayEquals(motors.getMotorsIndexesForSide(Motors.MotorSide.BACK), back);
    }

    @Test
    public void testReverseSide() {
        double[] input = {1, 1, 1, 1};
        this.motors.reverseMotorsOnSide(input, Motors.MotorSide.LEFT);
        assertArrayEquals(input, new double[]{-1, 1, 1, -1});
    }

    @Test
    public void testRotatePowers() {
        double[] motorPowers = {0, 0, 0, 0};

        // We are rotating counter clockwise so left should be negative and right should be positive
        this.motors.motorRotationPowers(0.5, motorPowers);
        assertArrayEquals(motorPowers, new double[]{-0.5, 0.5, 0.5, -0.5});

        motorPowers = new double[]{0, 0, 0, 0};
        this.motors.motorRotationPowers(-0.5, motorPowers);
        assertArrayEquals(motorPowers, new double[]{0.5, -0.5, -0.5, 0.5});
    }


}
