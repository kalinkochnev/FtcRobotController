package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.*;

//import android.graphics.Color;

public class Sensors {
    private final LinearOpMode opMode;
    HardwareMap hardwareMap;
    public Distance distance;
    public Color color;
//    public IMU imu;
    Sensors(LinearOpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.opMode = opMode;

        this.distance = new Distance();
//        this.imu= new IMU();
        this.color = new Color();
    }

    class IMU {
        private BNO055IMU imu;
        IMU() {
            initIMU();
        }
        public void initIMU() {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            while (!opMode.isStopRequested() && !imu.isGyroCalibrated()) {
                opMode.sleep(50);
                opMode.idle();
            }
        }

        public double getIMUAngleConverted() {
            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double angle = orientation.firstAngle;
            angle =  angle < 0 ? angle + 360 : angle;
            return angle;
        }
    }

    public class Distance {
        private DistanceSensor left;
        private DistanceSensor right;
        public DistanceUnit distanceUnit = DistanceUnit.METER;

        Distance() {
            left = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
            right = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
        }

        public double getLeftDistance() {
            return left.getDistance(distanceUnit);
        }

        public double getRightDistance() {
            return right.getDistance(distanceUnit);
        }
    }

    public class Color {
        public ColorSensor left;
        public ColorSensor right;

        Color() {
            left = hardwareMap.get(ColorSensor.class, "leftColorSensor");
            right = hardwareMap.get(ColorSensor.class, "rightColorSensor");
        }

//        public float[] getHSV(ColorSensor colorSensor) {
//            double a = colorSensor.alpha();
//            double r = colorSensor.red();
//            double g = colorSensor.green();
//            double b = colorSensor.blue();
//
//            float hsbValues[] = {0F, 0F, 0F};
//            final double SCALE_FACTOR = 255;
//
//            Color.RGBtoHSB(
//                    (int) (r * SCALE_FACTOR),
//                    (int) (g * SCALE_FACTOR),
//                    (int) (b * SCALE_FACTOR),
//                    hsbValues
//            );
//
//            return hsbValues;
//        }
    }
}
