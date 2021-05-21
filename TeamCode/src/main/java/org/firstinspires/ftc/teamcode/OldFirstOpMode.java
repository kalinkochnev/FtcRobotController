//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.helpers.Robot;
//
//
//@Autonomous(name = "Testing")
//public class FirstOpMode extends LinearOpMode {
//
//    private ElapsedTime runtime = new ElapsedTime();
//    private Robot robot;
//
//    @Override
//    public void runOpMode() {
//        this.robot = new Robot(this);
//
//        while (!this.isStarted()) {
//            continue;
//        }
//
//        this.robot.moveCardinal(0.2, 1, 90);
//        this.robot.moveCardinal(0.2, 1, 270);
//        this.robot.moveCardinal(0.2, 0.6*Math.sqrt(2), 45);
//        this.robot.moveCardinal(0.2, 0.6*Math.sqrt(2), 225);
//
//        while (!this.isStopRequested()) {
//            continue;
//        }
//    }
//
//
//
//}