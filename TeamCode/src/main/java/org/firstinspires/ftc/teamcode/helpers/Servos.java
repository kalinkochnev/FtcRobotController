package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servos {
    HardwareMap hardwareMap;
    Servo balloonPopper;

    Servos(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;

        // TODO this doesn't actually exist yet. Need to create in app settings.
//        this.balloonPopper = this.hardwareMap.get(Servo.class, "balloonPopper");

    }
}
