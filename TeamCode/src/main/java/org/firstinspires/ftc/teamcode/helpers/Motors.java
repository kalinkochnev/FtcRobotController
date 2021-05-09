package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.tntcorelib.util.SimplerHardwareMap;

public class Motors {
    public static final double GEAR_RATIO = 16.0/32.0; // output/input teeth
    public static final int TICKS_PER_MOTOR_ROTATION = 1120;
    public static final int TICKS_PER_WHEEL_ROTATION = (int)((TICKS_PER_MOTOR_ROTATION * GEAR_RATIO) + 0.5);
    public static final double WHEEL_DIAMETER = 0.1016; // [meters]
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // [meters]
    public static final double DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_WHEEL_ROTATION;

    // Modify this number so the proper distance is travelled
    public static final double DIST_MULTIPLIER = Math.sqrt(2); // This number was magically calculated empirically

    SimplerHardwareMap hardwareMap;

    protected DcMotor lfMotor, rfMotor, rbMotor, lbMotor;

    public Motors(SimplerHardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        lfMotor = this.hardwareMap.get(DcMotor.class, "lfMotor");
        rfMotor = this.hardwareMap.get(DcMotor.class,"rfMotor");
        lbMotor = this.hardwareMap.get(DcMotor.class,"lbMotor");
        rbMotor = this.hardwareMap.get(DcMotor.class,"rbMotor");

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

    /**
     * Sets powers for each motor. It uses configured motor directions and reverses the power to make it spin in the right
     * direction
     * @param power power for all motors
     */
    public void setPowers(double power) {
        setPowers(new double[]{power, power, power, power});
    }
    /**
     * Sets powers for each motor. It uses configured motor directions and reverses the power to make it spin in the right
     * direction
     * @param powers powers for each individual motors
     */
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
        int[] positions = new int[]{lfMotor.getCurrentPosition(), rfMotor.getCurrentPosition(), rbMotor.getCurrentPosition(), lbMotor.getCurrentPosition()};
        return this.applyMotorDirections(positions);
    }

    /**
     * This method returns an array of encoder positions that compensate for the power of the motor. So if a motor is going
     * at 0.5 power, and it travels at 10 ticks, at full power it would travel 20 ticks (or this is what I would like to believe,
     * no idea if this is actually true) // TODO make sure this works
     * This is useful for when you want to reach a certain distance but need to figure out if you have reached a certain
     * target distance.
     * @param motorPowers an array of motor powers to calculate the "virtual" ticks
     */
    public int[] getVirtualPositions(double[] motorPowers) {
        int[] virtualTicks = new int[4];
        int[] actualTicks = getMotorPositions();
        for (int motor = 0; motor < 4; motor++ ){
            virtualTicks[motor] = (int)((1-motorPowers[motor]) * actualTicks[motor]);
        }
        return virtualTicks;
    }



    /**
     * Returns array for directions of motors. This is used to configure what direction motors are supposed to spin.
     * A -1 implies reversed spin while +1 indicates as is. Configure this so that all wheels move in same direction
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
    public int[] applyMotorDirections(int[] motorEncoderTicks) {
        return Utils.toIntArr(this.applyMotorDirections(Utils.toDoubleArr(motorEncoderTicks)));
    }

    /**
     * Returns motor power weights which allow translation across a plane for a specified angle;
     * See https://seamonsters-2605.github.io/archive/mecanum/ for explanation of mecanum movement.
     * @param angle Angle, specified in radians where 0 is right.
     * @param powerMag The power magnitude of the movement (basically speed). Between 0 and 1.
     */
    public double[] translationWeights(double angle, double powerMag) {
        double[] weights = new double[4];
        double cornerSet1 = powerMag * Math.sin(angle + Math.PI / 4); // This is used for the front left wheel and bottom right wheel
        double cornerSet2 = powerMag * Math.sin(angle - Math.PI / 4); // This is used for the front right wheel and bottom left wheel

        weights[0] = cornerSet1; // Weight 0 refers to the front left wheel, and then going clockwise
        weights[1] = cornerSet2;
        weights[2] = cornerSet1;
        weights[3] = cornerSet2;
        return weights;
    }

    public PVector getNetEncoderVector() {
        return this.getNetEncoderVector(this.getMotorPositions());
    }
    public PVector getNetEncoderVector(int[] encoderPositions) {
        PVector[] wheelVectors = new PVector[4];

        double pi = Math.PI;
        double[] wheelDirections = {3*pi/4, pi/4, 3*pi/4, pi/4}; // This indicates the default direction the wheels act in when a power is applied to them

        for (int motor = 0; motor < 4; motor++) {
            wheelVectors[motor] = PVector.fromAngle(encoderPositions[motor], wheelDirections[motor]); // The distance spun can be negative, so should factor in if wheel is reversed
        }
        PVector resultant = PVector.add(wheelVectors).scalarDivide(2 * DIST_MULTIPLIER);
        return resultant; // Divide by two because vectors are really counted twice
    }

    /**This calculates the new motor powers to create a spin. Compensates for values > 1
     * @param angularPower the angular velocity to spin the robot at. Same as polar plane. + number implies counter clockwise, - clockwise.
     * @param translationalPower power magnitude of the *translation* movement. Between 0 and 1
     * @return new motor powers
     */
    private double[] motorRotationPowers(double angularPower, double[] powers) {

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
