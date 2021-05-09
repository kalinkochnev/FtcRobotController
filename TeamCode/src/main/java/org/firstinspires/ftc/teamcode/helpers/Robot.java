package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tntcorelib.util.RealSimplerHardwareMap;

import java.util.Arrays;

public class Robot {
    public Motors motors;
//    Sensors sensors;
    Telemetry telemetry;
    LinearOpMode opMode;

    public Robot(LinearOpMode opMode) {
        RealSimplerHardwareMap hardwareMap = new RealSimplerHardwareMap(opMode.hardwareMap);

        this.motors = new Motors(hardwareMap);
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;

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

//    public double getNearestMultipleOf90() {
//        double currentAngle = this.sensors.imu.getIMUAngleConverted();
//
//        double[] potentialValues = {0, 90, 180, 270};
//
//        double[] diffs = new double[potentialValues.length];
//
//        for (int i = 0; i < potentialValues.length; i++) {
//            diffs[i] = Math.abs(getAngleDifference(currentAngle, potentialValues[i]));
//        }
//
//        ArrayList<Double> diffsArrayList = new ArrayList<>();
//        for(double diff : diffs) {
//            diffsArrayList.add(diff);
//        }
//
//        double min = Collections.min(diffsArrayList);
//        int minIndex = diffsArrayList.indexOf(min);
//
//        return potentialValues[minIndex];
//    }

    public void holdAngleTest2(double maxPower, double distance, double angleHold) {
        this.motors.resetEncoders();
        angleHold = Math.toRadians(angleHold);

        double[] motorPowers = this.motors.translationWeights(angleHold, maxPower);
        this.motors.setPowers(motorPowers);

        double targetTicks = Motors.distanceToEncoderTicks(distance);
        int tickTravelled = 0;

        while (tickTravelled < targetTicks && !this.opMode.isStopRequested()) {
            telemetry.addData("target: ", targetTicks);
            telemetry.addData("Motor positions", Arrays.toString(this.motors.getMotorPositions()));

            tickTravelled = (int)(Math.round(this.motors.getNetEncoderVector().getMagnitude() / 2.828898391 * 100) / 100);
            telemetry.addData("ticks: ", tickTravelled);
            telemetry.update();
        }
        motors.setPowers(0);

    }

    // ------------------------------- Rotation -----------------------------------------------------------------------//
    public static double getAngleDifference(double currentAngle, double targetAngle) {

        currentAngle = currentAngle % 360;
        targetAngle = targetAngle % 360;

        currentAngle = currentAngle < 0 ? currentAngle + 360 : currentAngle;
        targetAngle = targetAngle < 0 ? targetAngle + 360 : targetAngle;

        double angleDiff = targetAngle - currentAngle;

        if (Math.abs(angleDiff) <= 180) {
            return angleDiff;
        }
        else {
            if (angleDiff > 0) {
                return angleDiff - 360;
            }
            else {
                return 360 + angleDiff;
            }
        }
    }

    public double getAngleDifferenceInDirection(String direction, double currentAngle, double targetAngle) {

        currentAngle = currentAngle % 360;
        targetAngle = targetAngle % 360;

        currentAngle = currentAngle < 0 ? currentAngle + 360 : currentAngle;
        targetAngle = targetAngle < 0 ? targetAngle + 360 : targetAngle;

        double angleDiff = targetAngle - currentAngle;

        if (direction.equals("ccw")) {
            if (angleDiff > 0) {
                return angleDiff;
            }
            else {
                return (360 - currentAngle) + targetAngle;
            }
        }
        else {
            if (angleDiff < 0) {
                return angleDiff;
            }
            else {
                return (targetAngle - 360) - currentAngle;
            }
        }
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
