package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tntcorelib.util.RealSimplerHardwareMap;

import java.util.Arrays;

public class Robot {
    private final Telemetry telemetry;
    private final LinearOpMode opMode;
    public Motors motors;
    public Sensors sensors;

    /**
     * Instantiates all motors and sensors to be used.
     *
     * @param opMode the LinearOpMode to be passed when instantiated in the opmode
     */
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
     * @param power     maximum speed of the robot
     * @param distance  distance to travel in meters
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
     * Strafes the robot in the direction of the specified angle at the power provided until powers are set back to 0.
     *
     * @param power     power/speed of robot during movement (between 0 and 1)
     * @param angleHold angle of movement for the robot (in degrees)
     * @return calculated powers to spin each wheel at to create the desired movement
     */
    public double[] moveCardinal(double power, double angleHold) {
        angleHold = Math.toRadians(angleHold);

        double[] motorPowers = this.motors.translationWeights(angleHold, power);
        this.motors.setPowers(motorPowers);
        return motorPowers;
    }

    /**
     * Strafes the robot in the direction of the specified angle at the maxPower provided for a certain distance.
     * Slowly ramps up/down speed of robot to prevent slippage.
     *
     * @param maxPower  greatest power/speed of robot during movement (between 0 and 1)
     * @param distance  distance to travel in specified direction (meters)
     * @param angleHold direction to translate towards (in degrees)
     */
    public void moveCardinal(double maxPower, double distance, double angleHold) {
        angleHold = Math.toRadians(angleHold);

        double targetTicks = Motors.distanceToEncoderTicks(distance); // Distance in ticks to the end goal


        double rampKickIn = 0.25; // Ramping kicks in at the first quarter of the drive and last quarter
        double rampOffset = 0.02;
        double rampupTicks = rampKickIn * targetTicks;

        double[] maxMotorPowers = this.motors.translationWeights(angleHold, maxPower);
        double[] currMotorPowers = new double[]{0, 0, 0, 0};

        int ticksTraveled = 0;

        this.motors.resetEncoders();
        while (ticksTraveled < targetTicks && !this.opMode.isStopRequested()) {
            ticksTraveled = (int) (Math.round(this.motors.getNetPositionVector().getMagnitude() * 100) / 100);


            double ticksTillTarget = targetTicks - ticksTraveled;
            telemetry.addData("Ticks till target: ", ticksTillTarget);
            // IF ramping
            if (ticksTraveled <= rampupTicks) { // If in the beginning percent of the movement to do start ramping
                telemetry.addData("Ramp status: ", "Up");
                for (int motor = 0; motor < 4; motor++) {
                    // The motor power scales proportionally to the distance already travelled
                    // So the further it travels, the faster it ramps up (to anyone who is interested, the ticks travelled
                    // grows exponentially/continuously during this time... yay calculus)
                    currMotorPowers[motor] = maxMotorPowers[motor] * (rampOffset + (ticksTraveled / rampupTicks));
                }
            } else if (ticksTillTarget <= rampupTicks) { // If in the last percent of the movement to do the end ramping
                telemetry.addData("Ramp status: ", "Down");
                for (int motor = 0; motor < 4; motor++) {
                    // Ramps down speed exponentially
                    currMotorPowers[motor] = maxMotorPowers[motor] - rampOffset * (-rampOffset + (ticksTillTarget / rampupTicks));
                }
            } else {
                telemetry.addData("Ramp status: ", "None");
                currMotorPowers = maxMotorPowers.clone();
            }
            telemetry.addData("Motor powers: ", Arrays.toString(currMotorPowers));
            telemetry.update();


            this.motors.setPowers(currMotorPowers);
        }
        motors.setPowers(0);

    }

    /**
     * Rotates the robot at the specified angular power until powers are set back to 0.
     *
     * @param angularPower Angular power to spin robot at. Negative angularPower means clockwise rotation.
     */
    public void rotate(double angularPower) {
        double[] rotationPowers = this.motors.motorRotationPowers(angularPower, new double[]{0, 0, 0, 0});
        this.motors.setPowers(rotationPowers);
    }

    /**
     * Rotates the robot at the specified angular power to reach a final angle
     *
     * @param angularPower Angular power to spin robot at.
     * @param angle        Angle (in degrees) to rotate from current orientation. Negative angle means clockwise rotation.
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
}
