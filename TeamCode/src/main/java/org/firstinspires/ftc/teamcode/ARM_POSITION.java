package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import android.bluetooth.BluetoothServerSocket;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name="ARM POSITION ")

public class ARM_POSITION extends OpMode {


    public Servo arm = null;


    public IMU imu = null;
    double initial_position = 0.0;


    public enum Intake_To_OutTake {
        START,
        ARM,
        DOWN_TO_PICK,
        ARM_ADJUST,
        LOCK_PIXELS,
        RESET_POSITION,
        OUTTAKE_POSITION,
        WAIT_TO_DROP_PIXEL,
    }

    Intake_To_OutTake intake_to_outtake = Intake_To_OutTake.START;
    ElapsedTime intake_to_outtake_timer = new ElapsedTime();
    final double close_time = 1;
    final double timer = 0.5;

    @Override
    public void init() {

        intake_to_outtake_timer.reset();

        // DRONE SERVO MAPPING & REVERSE


        //OUTTAKE SERVOS MAPPING
        arm = hardwareMap.get(Servo.class, "arm");

        //OUTAKE SERVOS REVERSE


        // INTAKE SERVOS MAPPING & REVERSAL


        //LINEAR SLIDE MOTORS MAPPING & REVERSE


        //MECANUM DRIVE MOTORS MAPPING


        //BRAKE FUNCTION


        //MECANUM DRIVE REVERSE


        //IMU LOCALIZATION
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        //INIT SERVO SETTING

        arm.setPosition(1);


    }

    @Override
    public void loop() {



        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Field Centric Drive Reset
        if (gamepad1.dpad_down) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bots rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // MECANUM DRIVE POWER


        // LINEAR SLIDE POWER
        // RAMP AND ROLLER INTAKE POWER`



        if (gamepad2.x && intake_to_outtake != Intake_To_OutTake.START) {
            intake_to_outtake = Intake_To_OutTake.START;
        }

        //OUTTAKE LEFT LOCK OPEN


        //OUTTAKE RIGHT LOCK OPEN


        //LOCK CLOSED


        // DRONE LAUNCHER

        if (gamepad1.a) {
            initial_position += 0.1;
            arm.setPosition(initial_position);
            telemetry.addData("Position", initial_position);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

        }
        if (gamepad1.y) {
            initial_position -= 0.1;
            arm.setPosition(initial_position);
            telemetry.addData("Position", initial_position);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }




    }
}

