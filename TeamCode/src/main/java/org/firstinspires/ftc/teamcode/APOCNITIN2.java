package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class APOCNITIN2 extends OpMode {

    @Override
    public void init() {
        Servo arm = hardwareMap.get(Servo.class, "arm");
    }

    @Override
    public void loop() {
        if (gamepad1.a){

        }

    }
}