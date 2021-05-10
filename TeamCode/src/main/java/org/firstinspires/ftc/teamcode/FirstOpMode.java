package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.helpers.Motors;
import org.firstinspires.ftc.teamcode.helpers.Robot;
import org.firstinspires.ftc.teamcode.helpers.Utils;
import org.firstinspires.ftc.teamcode.tntcorelib.util.RealSimplerHardwareMap;


@Autonomous(name="testing hold angle")
public class FirstOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot;
    @Override
    public void runOpMode() {
        this.robot = new Robot(this);

        while (!this.isStarted()) {
            continue;
        }

        this.robot.holdAngleTest2(0.25, 0.6 * Math.sqrt(2), 45);
        this.robot.holdAngleTest2(0.25, 0.6, 45);
        this.robot.holdAngleTest2(0.25, 0.6, 45);


//        double startAngle = 45;
//        int i = 0;
//        while (startAngle > 0) {
//            double newAngle = startAngle - i * 10;
//            this.robot.holdAngleTest2(0.35, 0.6, newAngle);
//            this.robot.holdAngleTest2(0.35, 0.6, 180 + newAngle);
//            i++;
//        }


        while (!this.isStopRequested()) {
            continue;
        }
    }



}