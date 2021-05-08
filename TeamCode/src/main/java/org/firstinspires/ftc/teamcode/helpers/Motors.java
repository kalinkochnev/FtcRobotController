package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Motors {
    public static final double GEAR_RATIO = 1; // output/input teeth
    public static final int TICKS_PER_MOTOR_ROTATION = 1120;
    public static final int TICKS_PER_WHEEL_ROTATION = (int)((TICKS_PER_MOTOR_ROTATION * GEAR_RATIO) + 0.5);
    public static final double WHEEL_DIAMETER = 0.1016; // [meters]
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // [meters]
    public static final double DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_WHEEL_ROTATION;

    HardwareMap hardwareMap;

    protected DcMotor lfMotor, rfMotor, rbMotor, lbMotor;

    public Motors(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;

        lfMotor = hardwareMap.dcMotor.get("lfMotor");
        rfMotor = hardwareMap.dcMotor.get("rfMotor");
        lbMotor = hardwareMap.dcMotor.get("lbMotor");
        rbMotor = hardwareMap.dcMotor.get("rbMotor");


//        this.rfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        this.rbMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.setAllZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.BRAKE);
        this.setAllRunModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * I am officially banishing the customary system from this repo
     * @param distance in meters
     * @return
     */
    public static int distanceToEncoderTicks(double distance) {
        return (int)((distance / DISTANCE_PER_TICK) + 0.5);
    }

    public static double encoderTicksToDistance(int ticks) { return ticks * DISTANCE_PER_TICK; }
    /**
     * This is a method that converts a desired distance into an "actual" distance to travel, which is based on how
     * the mecanum wheels interact with the surface. Often times there is slippage and the wheels fall short of their target
     * so this returns a compensated distance, which is often paired with the method distanceToEncoderTicks
     * @return
     */
    public static double compensateDistance(double desiredDistance) {
        /*How to calculate this constant:
        1. Turn on a pair of motors (on opposite corners) so the robot travel at a 45 degree angle. Make sure you set
        the robot to travel some test distance you expect it to reach (example, 5 meters at a 45 degree angle)
        2. Record how far the robot travelled side to side and forward and back under the variables realX and realY
        3. Repeat this trial multiple times and for opposite pairs of wheels, and take the average realX and realY and
        input it here
         */
        double expectedDist = 1; // This is the amount your expected it to travel in meters at a 45 degree angle

        // Do this trial multiple times
        final double realX = 0; // This is the amount the robot travelled side to side from the start pt to the end pt
        final double realY = 0; // This is the amount the robot travelled forward from start pt to end pt
        final double actualDist = Math.sqrt(Math.pow(realX, 2) + Math.pow(realY,2));

        return desiredDistance * (expectedDist/actualDist);
    }

    public DcMotor[] getMotors() {
        return new DcMotor[]{lfMotor, rfMotor, rbMotor, lbMotor};
    }

    public void resetEncoders() {
        this.setAllRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setAllRunModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    private void setAllRunModes(DcMotor.RunMode runMode) {
        for (DcMotor motor : getMotors()) {
            motor.setMode(runMode);
        }
    }

    public void setPowers(double power) {
        setPowers(new double[]{power, power, power, power});
    }
    public void setPowers(double[] powers) {
        powers = this.applyMotorDirections(powers);
        DcMotor[] motors = getMotors();
        for (int i = 0; i < 4; i++) {
            motors[i].setPower(powers[i]);
        }
    }

    private void setAllZeroPowerBehaviors(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotor motor : getMotors()) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }


    /**
     * Retrieves motor encoder positions. Follows motor index convention: First index refers to front left wheel, and goes clockwise
     */
    public int[] getMotorPositions() {
        return new int[]{lfMotor.getCurrentPosition(), rfMotor.getCurrentPosition(), lbMotor.getCurrentPosition(), rbMotor.getCurrentPosition()};
    }

//    /**
//     * This method returns an array of encoder positions that compensate for the power of the motor. So if a motor is going
//     * at 0.5 power, and it travels at 10 ticks, at full power it would travel 20 ticks (or this is what I would like to believe,
//     * no idea if this is actually true) // TODO make sure this works
//     * This is useful for when you want to reach a certain distance but need to figure out if you have reached a certain
//     * target distance.
//     * @param motorPowers an array of motor powers to calculate the "virtual" ticks
//     */
//    public int[] getVirtualPositions(double[] motorPowers, ) {
//        int[] virtualTicks = new int[4];
//        int[] actualTicks = getMotorPositions();
//        for (int motor = 0; motor < 4; motor++ ){
//            virtualTicks[motor] = (int)((1/motorPowers[motor]) * actualTicks[motor]);
//        }
//        return virtualTicks;
//    }
//
    public int[] diffInPositions(int[] oldPositions) {
        int[] diff = new int[4];
        int[] newPositions = getMotorPositions();

        for (int motor = 0; motor < 4; motor++) {
            diff[motor] = newPositions[motor] - oldPositions[motor];
        }
        return diff;
    }

    /**
     * Returns array for directions of motors. This is used to configure what direction motors are supposed to spin.
     * A -1 implies reversed spin while +1 indicates as is. Configure this so that the robot moves forward when you
     * set all the powers to some value.
     * @return An array of -1 or 1's.
     */
    public int[] getMotorDirections() {
        int[] directions = {
                -1, 1, 1, -1 // Front left, Front Right, Back Right, Back Left
        };
        return directions;
    }

    /**
     * Applies motor directions (reversed or not reversed) to array with motor powers.
     * @param motorPowers array of motor powers
     * @return Motor powers with motor directions applied
     */
    public double[] applyMotorDirections(double[] motorPowers) {
        int[] directions = getMotorDirections();
        for (int m = 0; m < 4; m++) {
            motorPowers[m] *= directions[m];
        }
        return motorPowers;
    }

    /**
     * Returns motor power weights which allow translation across a plane for a specified angle;
     * See https://seamonsters-2605.github.io/archive/mecanum/ for explanation of mecanum movement.
     * @param angle Angle, specified in radians where 0 is right.
     * @param powerMag The power magnitude of the movement (basically speed). Between 0 and 1.
     */
    public double[] motorTranslationWeights(double angle, double powerMag) {
        double[] weights = new double[4];
        double cornerSet1 = powerMag * Math.sin(angle + Math.PI / 4); // This is used for the front left wheel and bottom right wheel
        double cornerSet2 = powerMag * Math.sin(angle - Math.PI / 4); // This is used for the front right wheel and bottom left wheel

        weights[0] = cornerSet1; // Weight 0 refers to the front left wheel, and then going clockwise
        weights[1] = cornerSet2;
        weights[2] = cornerSet1;
        weights[3] = cornerSet2;
        return weights;
    }

    /**This calculates the new motor powers on top of translation movement to create a spin. Compensates for values > 1
     * @param angularPower the angular velocity to spin the robot at. Same as polar plane. + number implies counter clockwise, - clockwise.
     * @param translationalPower power magnitude of the *translation* movement. Between 0 and 1
     * @return new motor powers
     */
    private double[] motorRotationPowers(double angularPower, double translationalPower) {
        double[] powers = motorTranslationWeights(angularPower, translationalPower);

        // TODO possibly need to adjust sign of power for turns
        // Add the rotational power to translational power. See https://seamonsters-2605.github.io/archive/mecanum/ for explanation
        for (int motor = 0; motor < 4; motor++) {
            powers[motor] += angularPower;
        }

        // Divide by the max value power to scale value from -1 to 1
        double absMaxPower = Utils.getMax(Utils.absoluteValues(powers));
        if (absMaxPower != 0) {
            for (int motor = 0; motor < 4; motor++) {
                powers[motor] /= absMaxPower;
            }
        }

        return powers;
    }


}
