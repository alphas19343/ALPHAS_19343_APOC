package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp (name = "MotorServoPositionTest")
public class MotorServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization Code Goes Here



        //DRONE LAUNCHER & REVERSE
        Servo drone = hardwareMap.get(Servo.class, "drone");
        drone.setDirection(Servo.Direction.REVERSE);

        // INTAKE ARM, ROLLERS & RAMP MAPPING
        Servo intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        Servo intakeRight = hardwareMap.get(Servo.class,"intakeRight");
        DcMotor rollers = hardwareMap.get(DcMotor.class, "rollers");
        CRServo ramp = hardwareMap.get(CRServo.class, "ramp");

        // INTAKE ARM REVERSE
        intakeLeft.setDirection(Servo.Direction.REVERSE);

        //OUTAKE ARM, ROTATE, LS AND LOCKS
        Servo LSSLeft =  hardwareMap.get(Servo.class,"LSSLeft");
        Servo LSSRight = hardwareMap.get(Servo.class, "LSSRight");
        Servo arm = hardwareMap.get(Servo.class, "arm");
        Servo rotate = hardwareMap.get(Servo.class, "rotate");
        Servo lockLeft = hardwareMap.get(Servo.class, "lockLeft");
        Servo lockRight = hardwareMap.get(Servo.class, "lockRight");

        //OUTAKE REVERSAL
        lockLeft.setDirection(Servo.Direction.REVERSE);
        LSSLeft.setDirection(Servo.Direction.REVERSE);
        rotate.setDirection(Servo.Direction.REVERSE);

        // LINEAR SLIDES MOTOR MAPPING
        DcMotor rightLSM = hardwareMap.get(DcMotor.class,"rightLSM");
        DcMotor leftLSM = hardwareMap.get(DcMotor.class, "leftLSM");

        // LINEAR SLIDE MOTOR REVERSE
        rightLSM.setDirection(DcMotorSimple.Direction.REVERSE);
        rollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightLSM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftLSM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // MECANUM DRIVE MOTORS MAPPING
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        // WHEELS DIRECTION REVERSE
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU MAPPING & ORIENTATION
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        // Define GAMEPADS & VARIABLES
        boolean togglerb = true;
        boolean togglelb = true;
        boolean togglea = true;
        boolean toggley = true;
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        //INIT SERVO SETTING
        LSSLeft.setPosition(0.85);
        LSSRight.setPosition(0.85);
        arm.setPosition(0.9);
        rotate.setPosition(0.48);
        lockLeft.setPosition(0);
        lockRight.setPosition(0);
        drone.setPosition(0.01);

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (currentGamepad2.a && !previousGamepad2.a) {
                togglea = !togglea;
            }
            if (togglea){
                LSSLeft.setPosition(0.94);
                LSSRight.setPosition(0.94);
            } else {
                LSSLeft.setPosition(0);
                LSSRight.setPosition(0);
            }

            if (currentGamepad2.y && !previousGamepad2.y){
                toggley = !toggley;
            }
            if (toggley){
                arm.setPosition(0.8);
            } else {
                arm.setPosition(0.12);
            }

            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper){
                togglerb = !togglerb;
            }
            if (togglerb){
                lockLeft.setPosition(0);
            } else {
                lockLeft.setPosition(0.55);
            }

            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper){
                togglelb = !togglelb;
            }
            if (togglelb){
                lockRight.setPosition(0);
            } else {
                lockRight.setPosition(0.55);
            }
            // ROTATE SERVO SETTING
            if (gamepad1.a) {
                rotate.setPosition(0.14);
            }
            if (gamepad1.x) {
                rotate.setPosition(0.33);
            }
            if (gamepad1.y) {
                rotate.setPosition(0.8);
            }
            if (gamepad1.b) {
                rotate.setPosition(1);
            }
            if (gamepad1.left_bumper) {
                rotate.setPosition(0.49);
            }
        }
    }
}