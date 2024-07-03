package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "locktest")

public class locktest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo lockLeft = hardwareMap.get(Servo.class, "lockLeft");
        Servo lockRight = hardwareMap.get(Servo.class, "lockRight");

        lockLeft.setDirection(Servo.Direction.REVERSE);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        // Define GAMEPADS & VARIABLES
        boolean togglerb = true;
        boolean togglelb = true;
        boolean togglea = true;
        boolean toggley = true;

        //INIT SERVO SETTING
        lockLeft.setPosition(0);
        lockRight.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                togglerb = !togglerb;
            }
            if (togglerb) {
                lockRight.setPosition(0);
            } else {
                lockRight.setPosition(0.55);
            }

            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                togglelb = !togglelb;
            }
            if (togglelb) {
                lockLeft.setPosition(0);
            } else {
                lockLeft.setPosition(0.55);
            }

        }
    }
}
