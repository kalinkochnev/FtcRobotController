package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.helpers.Robot;


@Autonomous(name = "Testing")
public class FirstOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot;
    @Override
    public void runOpMode() {
        this.robot = new Robot(this);

        while (!this.isStarted()) {
            continue;
        }

        this.robot.rotate(0.1, 180);
        this.robot.rotate(0.1, -180);

        while (!this.isStopRequested()) {
            continue;
        }
    }



}