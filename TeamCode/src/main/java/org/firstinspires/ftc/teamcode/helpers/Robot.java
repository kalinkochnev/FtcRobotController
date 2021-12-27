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
    public Servos servos;

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
        this.servos = new Servos(opMode);
    }


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
     * @param angleHold direction to translate towards (in degrees). Follows unit circle. 90 degrees is forward.
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
