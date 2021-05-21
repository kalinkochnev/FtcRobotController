package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.helpers.MoveSequence;
import org.firstinspires.ftc.teamcode.helpers.Robot;

@Autonomous(name = "FirstOpMode")
public class FirstOpMode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot;
    
    // double[][] inputs = {
    //     {17, 0},
    //     {77, 270}
    // };

    double[][] startToBalloonInputs = {
       {3,270},
       {120,0} 
    };

    // private MoveSequence spikeToDrop = new MoveSequence({
    //     {67, 90},
    //     {69, 0},
    //     {69, 270},
    //     {0, 90},
    //     {80, 270},
    //     {20, 180}
    // });
    
    double[][] balloonPopInputs = {
        {10, 90},
        {10, 270},
        {4, 180}
    };

    
    @Override
    public void runOpMode() {
        this.robot = new Robot(this);
        MoveSequence startToBalloon = new MoveSequence(robot, startToBalloonInputs);
        MoveSequence balloonPop = new MoveSequence(robot, balloonPopInputs);

        while (!this.isStarted()) {
            continue;
        }

        startToBalloon.execute();

        for (int i = 0; i < 4; i++) {
            balloonPop.execute();
        }
    
    }       
}