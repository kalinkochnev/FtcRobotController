package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.helpers.Robot;


@Autonomous(name="testing hold angle")
public class FirstOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot;
    @Override
    public void runOpMode() {
        this.robot = new Robot(this);

        while (!this.isStarted()) {
            telemetry.addData("Status", "Waiting for start");
            telemetry.addData("Runtime", runtime.toString());
            telemetry.update();
        }

        this.robot.motors.setPowers(0.25);
        sleep(10000);
    }



}