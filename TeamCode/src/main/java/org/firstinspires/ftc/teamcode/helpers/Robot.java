package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tntcorelib.util.RealSimplerHardwareMap;

import java.util.Arrays;

public class Robot {
    public Motors motors;
    public Sensors sensors;
    Telemetry telemetry;
    LinearOpMode opMode;

    public Robot(LinearOpMode opMode) {
        RealSimplerHardwareMap hardwareMap = new RealSimplerHardwareMap(opMode.hardwareMap);

        this.motors = new Motors(hardwareMap);
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;
        this.sensors = new Sensors(opMode);

    }
    /**
     * This automatically tries to maintain a translational angle while strafing and rotating (if used) for a certain
     * distance. Also ramps up/down as it starts/ends it's movement to be more accurate.
     * @param power maximum speed of the robot
     * @param distance distance to travel in meters
     * @param direction direction to translate towards (in degrees)
     */
//    public void holdAngle(double power, double distance, double direction) {
//        double angleHold = getNearestMultipleOf90();
//        this.motors.resetEncoders();
//
//        int[] motorDirections = this.motors.getMotorDirections();
//
//        if (direction == 0) {
//            motorDirections[0] *= -1;
//            motorDirections[1] *= -1;
//        }
//        else if (direction == 90) {
//            motorDirections[0] *= -1;
//            motorDirections[2] *= -1;
//        }
//        else if (direction == 180) {
//            motorDirections[2] *= -1;
//            motorDirections[3] *= -1;
//        }
//        else if (direction == 270) {
//            motorDirections[1] *= -1;
//            motorDirections[3] = -1;
//        }
//        else {
//            throw new RuntimeException("Direction must be -1 or 1");
//        }
//
//        if (direction == 0 || direction == 180) {
//            distance *= ((304.8) / (304.8 - 19));
//        }
//
//        double targetTicks = Motors.distanceToEncoderTicks(distance);
//        int averageMotorTicks = 0;
//
//        boolean useRamping = true;
//        double rampupTicks = 500 * power;
//        if (targetTicks / 2 < rampupTicks) {
//            rampupTicks = targetTicks / 2;
//        }
//
//        while (averageMotorTicks < targetTicks && !this.opMode.isStopRequested()) {
//            averageMotorTicks = (int)(Utils.getSum(Utils.absoluteValues(this.motors.getMotorPositions())) / 4);
//
//            double motorPower = power;
//
//            if (useRamping) {
//                double powerOffsetStart = 0.05;
//                double powerOffsetEnd = 0.05;
//
//                power = Math.max(power, Math.max(powerOffsetEnd, powerOffsetStart));
//
//                if (averageMotorTicks < rampupTicks) {
//                    motorPower = powerOffsetStart + (power - powerOffsetStart) * (averageMotorTicks / rampupTicks);
//                }
//                else if (averageMotorTicks > targetTicks - rampupTicks) {
//                    motorPower = powerOffsetEnd + (power - powerOffsetEnd) * (targetTicks - averageMotorTicks) / rampupTicks;
//                }
//                else {
//                    motorPower = power;
//                }
//            }
//
//            double tmpAngle = this.sensors.imu.getIMUAngleConverted();
//            double angle = getAngleDifference(tmpAngle, angleHold);
//
//            double k = 0.04;
//            double correction = angle * k;
//
//            double[] motorPowers = {
//                    motorPower * motorDirections[0],
//                    motorPower * motorDirections[1],
//                    motorPower * motorDirections[2],
//                    motorPower * motorDirections[3],
//            };
//
//            double minPower = getMin(motorPowers);
//            double maxPower = getMax(motorPowers);
//
//            double tempMax = Math.max(Math.abs(minPower), Math.abs(maxPower));
//            double tempCorrectedMax = Math.max(Math.abs(minPower + correction), Math.abs(maxPower + correction));
//
//            if (tempCorrectedMax > 1) {
//                for (int i = 0; i < motorPowers.length; i++) {
//                    motorPowers[i] *= (1 - Math.abs(correction)) / tempMax;
//                }
//            }
//
//            for (int i = 0; i < motorPowers.length; i++) {
//                motorPowers[i] += correction;
//            }
//
//            this.motors.setPowers(motorPowers);
//        }
//
//        this.motors.setPowers(new double[]{0, 0, 0, 0});
//    }

    /**
     * Strafes the robot in the direction of the specified angle at the maxPower provided
     *
     * @param power
     * @param angleHold
     */
    public void moveCardinal(double power, double angleHold) {
        angleHold = Math.toRadians(angleHold);

        double[] motorPowers = this.motors.translationWeights(angleHold, power);
        this.motors.setPowers(motorPowers);
    }

    /**
     * Strafes the robot in the direction of the specified angle at the maxPower provided for a certain distance.
     *
     * @param maxPower
     * @param angleHold
     */
    public void moveCardinal(double maxPower, double distance, double angleHold) {
        this.motors.resetEncoders();
        this.moveCardinal(maxPower, angleHold);

        double targetTicks = Motors.distanceToEncoderTicks(distance);
        int tickTravelled = 0;

        while (tickTravelled < targetTicks && !this.opMode.isStopRequested()) {
            tickTravelled = (int) (Math.round(this.motors.getNetPositionVector().getMagnitude() * 100) / 100);
        }
        motors.setPowers(0);

    }

    /**
     * Rotates the robot at the specified angular power. Negative angular means counter clockwise rotation.
     *
     * @param angularPower
     */
    public void rotate(double angularPower) {
        double[] rotationPowers = this.motors.motorRotationPowers(angularPower, new double[]{0, 0, 0, 0});
        this.motors.setPowers(rotationPowers);
    }

    /**
     * Rotates the robot at the specified angular power to reach a final angle
     *
     * @param angularPower
     * @param angle
     */
    public void rotate(double angularPower, double angle) {
        this.motors.resetEncoders();

        double currentAngle = this.sensors.imu.getAngleConverted();
        double targetAngle = currentAngle + angle;

        angularPower = Math.copySign(angularPower, angle); // Copy the sign of the angle and apply it to the power to set rotation direction
        this.rotate(angularPower);

        while (Math.abs(currentAngle - targetAngle) > 0 && !this.opMode.isStopRequested()) {
            currentAngle = this.sensors.imu.getAngleConverted();
        }
        this.motors.setPowers(0);
    }

    /**
     * This automatically tries to maintain a specified angle when rotating. Also ramps up/down as it starts/ends it's movement to be more accurate.
     * @param rotationMaxPow max speed of the robot during rotation
     * @param targetAngle angle to try and spin to
     * @param forceDirection  // TODO I have no idea what this does yet
     * @param direction // TODO I have no idea what this does yet
     */
//    public void rotateAngle(double rotationMaxPow, double targetAngle, boolean forceDirection, String direction) {
//
//        // Direction is either "ccw" or "cw", else don't force direction
//
//        double angleTolerance = 2;
//        double timeUnderToleance = 200;
//        boolean inTolerance = false;
//
//        ElapsedTime timer = new ElapsedTime();
//
//        while (!this.opMode.isStopRequested()) {
//
//            double tmpAngle = this.sensors.imu.getIMUAngleConverted();
//            double angle = getAngleDifference(tmpAngle, targetAngle);
//            double angleDir = getAngleDifferenceInDirection(direction, tmpAngle, targetAngle);
//
//            if (forceDirection) {
//                if (Math.abs(angleDir) >= 180) {
//                    angle = angleDir;
//                }
//            }
//
//            if (Math.abs(angle) < angleTolerance) {
//                if (!inTolerance) {
//                    timer.reset();
//                }
//                inTolerance = true;
//                if (timer.milliseconds() > timeUnderToleance) {
//                    break;
//                }
//            }
//            else {
//                inTolerance = false;
//            }
//
//            double k = 0.04;
//            double correction = angle * k;
//
//            double motorPower = Math.min(correction, rotationMaxPow);
//            this.motors.setPowers(motorPower);
//        }
//        this.motors.setPowers(0);
//    }
}
