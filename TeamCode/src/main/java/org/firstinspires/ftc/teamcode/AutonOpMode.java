package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.helpers.MoveSequence;
import org.firstinspires.ftc.teamcode.helpers.Robot;

@Autonomous(name = "Testing")
public class AutonOpMode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot;
    double[][] inputs = {
            {17, 270},
            {77, 180}
    };


    // private MoveSequence spikeToDrop = new MoveSequence({
    //     {67, 0},
    //     {69, 270},
    //     {69, 180},
    //     {0, 90},
    //     {80, 180},
    //     {20, 90}
    // });

    // private MoveSequence dropToStart = new MoveSequence ({
    //     {69, 0},
    //     {},
    // });

    // private MoveSequence balloonPop = new MoveSequence ({
    //     {},
    //     {},
    // });


    @Override
    public void runOpMode() {
        this.robot = new Robot(this);
        MoveSequence startToSpike = new MoveSequence(robot, inputs);

        while (!this.isStarted()) {
            continue;
        }

        startToSpike.execute();

        // this.robot.servos.gateCloser.setPosition();
        // spikeToDrop.execute();
        // droptoStart.execute();

        // spikeToDrop.execute();

        while (!this.isStopRequested()) {
            continue;
        }
    }
}