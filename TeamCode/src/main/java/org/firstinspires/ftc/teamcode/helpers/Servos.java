package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servos {
    HardwareMap hardwareMap;
    Servo gateCloser;

    Servos(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.gateCloser = this.hardwareMap.get(Servo.class, "gateCloser");

    }
}
