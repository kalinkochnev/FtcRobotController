package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.tntcorelib.util.SimplerHardwareMap;


public class Motors {
    public static final double GEAR_RATIO = 16.0/32.0; // output/input teeth
    public static final int TICKS_PER_MOTOR_ROTATION = 1120;
    public static final int TICKS_PER_WHEEL_ROTATION = (int)((TICKS_PER_MOTOR_ROTATION * GEAR_RATIO) + 0.5);
    public static final double WHEEL_DIAMETER = 0.1016; // [meters]
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // [meters]
    public static final double DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_WHEEL_ROTATION;

    // Modify this number so the proper distance is travelled
    public static final double DIST_MULTIPLIER = Math.sqrt(2); // This number was magically calculated empirically. By luck it was sqrt(2)

    SimplerHardwareMap hardwareMap;

    protected DcMotor lfMotor, rfMotor, rbMotor, lbMotor;

    public enum MotorSide {
        LEFT, FRONT, RIGHT, BACK
    }

    public Motors(SimplerHardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        lfMotor = this.hardwareMap.get(DcMotor.class, "lfMotor");
        rfMotor = this.hardwareMap.get(DcMotor.class, "rfMotor");
        lbMotor = this.hardwareMap.get(DcMotor.class, "lbMotor");
        rbMotor = this.hardwareMap.get(DcMotor.class, "rbMotor");

        this.setAllZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.BRAKE);
        this.setAllRunModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * I am officially banishing the customary system from this repo
     * @param distance in meters
     * @return
     */
    public static int distanceToEncoderTicks(double distance) {
        return (int) ((distance / DISTANCE_PER_TICK) + 0.5);
    }

    public static double encoderTicksToDistance(int ticks) {
        return ticks * DISTANCE_PER_TICK;
    }

    public DcMotor[] getMotors() {
        return new DcMotor[]{lfMotor, rfMotor, rbMotor, lbMotor};
    }

    public double[] getMotorPowers() {
        double[] powers = new double[4];
        DcMotor[] motors = this.getMotors();
        for (int motor = 0; motor < 4; motor++) {
            powers[motor] = motors[motor].getPower();
        }
        return powers;
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

    /**
     * Returns indexes of motors to get for that side
     *
     * @param side
     * @return
     */
    public int[] getMotorsIndexesForSide(MotorSide side) {
        int startIndex = 0; // The start index is the index of the first motor on the specified side
        switch (side) {
            case LEFT:
                startIndex = 3;
                break;
            case FRONT:
                startIndex = 0;
                break;
            case RIGHT:
                startIndex = 1;
                break;
            case BACK:
                startIndex = 2;
                break;
        }
        return new int[]{startIndex, (startIndex + 1) % 4}; // This loops back to first motor so array not out of bounds
    }

    private void setAllZeroPowerBehaviors(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotor motor : getMotors()) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }


    /**
     * Retrieves motor encoder positions that accounts for motor directions. Follows motor index convention: First index
     * refers to front left wheel, and goes clockwise
     */
    public int[] getMotorPositions() {
        int[] positions = new int[]{lfMotor.getCurrentPosition(), rfMotor.getCurrentPosition(), rbMotor.getCurrentPosition(), lbMotor.getCurrentPosition()};
        return this.applyMotorDirections(positions);
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
        double[] powers = motorPowers.clone();
        for (int m = 0; m < 4; m++) {
            powers[m] *= directions[m];
        }
        return powers;
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

    /**
     * This adds up the vector positions that the movement of the wheels creates. Can specify encoder positions or
     * default uses the current encoder positions.
     *
     * @return PVector of the robot's current position since the last encoder reset by default
     */
    public PVector getNetPositionVector() {
        return this.getNetPositionVector(this.getMotorPositions());
    }

    public PVector getNetPositionVector(int[] encoderPositions) {
        PVector[] wheelVectors = new PVector[4];

        double pi = Math.PI;
        // This indicates the default direction the wheels act in when a positive power is applied to them. If this is not the
        // case, configure the motor directions in getMotorDirections()
        double[] wheelDirections = {3 * pi / 4, pi / 4, 3 * pi / 4, pi / 4};

        for (int motor = 0; motor < 4; motor++) {
            wheelVectors[motor] = PVector.fromAngle(encoderPositions[motor], wheelDirections[motor]); // The distance spun can be negative, so should factor in if wheel is reversed
        }

        // Divide by two because vectors are really counted twice and divide by sqrt(2) b/c we want to have the same magnitude
        // as the individual vectors. This is possible because of the symmetry of the force vectors from the mecanum wheels
        PVector resultant = PVector.add(wheelVectors).scalarDivide(2 * Math.sqrt(2));
        return resultant;
    }


    public double[] reverseMotorsOnSide(double[] powers, MotorSide side) {
        int[] sideIndexes = this.getMotorsIndexesForSide(side);
        powers[sideIndexes[0]] *= -1;
        powers[sideIndexes[1]] *= -1;
        return powers;
    }

    /**
     * This calculates the new motor powers to create a spin. Compensates for values > 1.
     *
     * @param angularPower the angular velocity to spin the robot at. Same as polar plane. + number implies counter clockwise, - clockwise.
     * @param powers       the existing powers to apply a rotation to
     * @return new motor powers
     */
    public double[] motorRotationPowers(double angularPower, double[] powers) {
        // If we want to go clockwise, we speed up left motors and right motors (in opp direction), and vice versa
        double[] powToAdd = {angularPower, angularPower, angularPower, angularPower};

        // Angular power is already negative when you want to go clockwise (right motors are reversed), so either way
        // you can just reverse the left motors regardless of clockwise vs counter-clockwise
        this.reverseMotorsOnSide(powToAdd, MotorSide.LEFT);

        for (int motor = 0; motor < 4; motor++) {
            powers[motor] += powToAdd[motor];
        }


        // Divide by the max value power to scale value from -1 to 1 if exceeds 1
        double absMaxPower = Utils.getMax(Utils.absoluteValues(powers));
        if (absMaxPower > 1) {
            for (int motor = 0; motor < 4; motor++) {
                powers[motor] /= absMaxPower;
            }
        }

        return powers;
    }


}
