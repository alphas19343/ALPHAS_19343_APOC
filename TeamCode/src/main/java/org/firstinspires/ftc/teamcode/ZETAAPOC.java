package org.firstinspires.ftc.teamcode;

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

@TeleOp (name="ZETA APOC")

public class ZETAAPOC extends OpMode {
    public Servo LSSLeft = null;
    public Servo LSSRight = null;

    public Servo arm = null;

    public Servo rotate = null;
    public Servo lockLeft = null;
    public Servo lockRight = null;
    public Servo drone = null;

    public Servo intakeLeft = null;
    public Servo intakeRight = null;
    public DcMotor rollers = null;
    public CRServo ramp = null;
    public DcMotor rightLSM = null;
    public DcMotor leftLSM = null;
    public DcMotor leftBack = null;
    public DcMotor leftFront = null;
    public DcMotor rightBack = null;
    public DcMotor rightFront = null;

    public IMU imu = null;


    public enum Intake_To_OutTake {
        START,
        ARM,
        DOWN_TO_PICK,
        LOCK_PIXELS,
        RESET_POSITION,
        OUTTAKE_POSITION,
        WAIT_TO_DROP_PIXEL,
    }

    Intake_To_OutTake intake_to_outtake = Intake_To_OutTake.START;
    ElapsedTime intake_to_outtake_timer = new ElapsedTime();
    final double close_time = 1;

    @Override
    public void init() {

        intake_to_outtake_timer.reset();

        // DRONE SERVO MAPPING & REVERSE
        drone = hardwareMap.get(Servo.class, "drone");
        drone.setDirection(Servo.Direction.REVERSE);

        //OUTTAKE SERVOS MAPPING
        LSSLeft = hardwareMap.get(Servo.class, "LSSLeft");
        LSSRight = hardwareMap.get(Servo.class, "LSSRight");
        arm = hardwareMap.get(Servo.class, "arm");
        rotate = hardwareMap.get(Servo.class, "rotate");
        lockLeft = hardwareMap.get(Servo.class, "lockLeft");
        lockRight = hardwareMap.get(Servo.class, "lockRight");

        //OUTAKE SERVOS REVERSE
        lockLeft.setDirection(Servo.Direction.REVERSE);
        LSSLeft.setDirection(Servo.Direction.REVERSE);
        rotate.setDirection(Servo.Direction.REVERSE);

        // INTAKE SERVOS MAPPING & REVERSAL
        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        rollers = hardwareMap.get(DcMotor.class, "rollers");
        ramp = hardwareMap.get(CRServo.class, "ramp");
        intakeLeft.setDirection(Servo.Direction.REVERSE);

        //LINEAR SLIDE MOTORS MAPPING & REVERSE
        rightLSM = hardwareMap.get(DcMotor.class, "rightLSM");
        leftLSM = hardwareMap.get(DcMotor.class, "leftLSM");
        rightLSM.setDirection(DcMotorSimple.Direction.REVERSE);
        rollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //MECANUM DRIVE MOTORS MAPPING
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        //BRAKE FUNCTION
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //MECANUM DRIVE REVERSE
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //IMU LOCALIZATION
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        //INIT SERVO SETTING
        LSSLeft.setPosition(0.85);
        LSSRight.setPosition(0.85);
        arm.setPosition(0.8);
        rotate.setPosition(0.51);
        lockLeft.setPosition(0);
        lockRight.setPosition(0);
        drone.setPosition(0.01);

    }

    @Override
    public void loop() {

        switch (intake_to_outtake) {

            case START:
                if (gamepad2.a) {
                    arm.setPosition(0.8);
                    LSSLeft.setPosition(0.94);
                    LSSRight.setPosition(0.94);
                    rotate.setPosition(0.51);
                    lockLeft.setPosition(0);
                    lockRight.setPosition(0);
                    intake_to_outtake_timer.reset();
                    intake_to_outtake = Intake_To_OutTake.ARM;
                }
                break;

            case ARM:
                if (intake_to_outtake_timer.seconds() >= close_time) {
                    LSSLeft.setPosition(0.85);
                    LSSRight.setPosition(0.85);
                    intake_to_outtake_timer.reset();
                    intake_to_outtake = Intake_To_OutTake.DOWN_TO_PICK;
                }
                break;

            case DOWN_TO_PICK:
                if (gamepad2.y) {
                    LSSLeft.setPosition(0.94);
                    LSSRight.setPosition(0.94);
                    intake_to_outtake_timer.reset();
                    intake_to_outtake = Intake_To_OutTake.LOCK_PIXELS;
                }
                break;

            case LOCK_PIXELS:
                if (intake_to_outtake_timer.seconds() >= close_time) {
                    lockRight.setPosition(0.6);
                    lockLeft.setPosition(0.55);
                    intake_to_outtake_timer.reset();
                    intake_to_outtake = Intake_To_OutTake.RESET_POSITION;
                }
                break;

            case RESET_POSITION:
                if (intake_to_outtake_timer.seconds() >= close_time) {
                    LSSLeft.setPosition(0.85);
                    LSSRight.setPosition(0.85);
                    arm.setPosition(0.8);
                    rotate.setPosition(0.51);
                    intake_to_outtake_timer.reset();
                    intake_to_outtake = Intake_To_OutTake.OUTTAKE_POSITION;
                }
                break;

            case OUTTAKE_POSITION:
                if (gamepad2.a) {
                    LSSLeft.setPosition(0);
                    LSSRight.setPosition(0);
                    arm.setPosition(0.12);
                    rotate.setPosition(0.51);
                    intake_to_outtake_timer.reset();
                    intake_to_outtake = Intake_To_OutTake.WAIT_TO_DROP_PIXEL;
                }
                break;

            case WAIT_TO_DROP_PIXEL:
                if (intake_to_outtake_timer.seconds() >= close_time) {
                    intake_to_outtake_timer.reset();
                    intake_to_outtake = Intake_To_OutTake.START;
                }
                    break;

                default:
                    intake_to_outtake = Intake_To_OutTake.START;
        }

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
                rightFront.setPower(frontRightPower * 0.7);
                leftFront.setPower(frontLeftPower * 0.7);
                rightBack.setPower(backRightPower * 0.7);
                leftBack.setPower(backLeftPower * 0.7);

                // LINEAR SLIDE POWER
                rightLSM.setPower(gamepad2.left_stick_y);
                leftLSM.setPower(gamepad2.left_stick_y);

                // RAMP AND ROLLER INTAKE POWER`
                rollers.setPower(-gamepad2.right_stick_y * 0.5);
                ramp.setPower(gamepad2.right_stick_y);

                if (gamepad2.right_stick_y != 0) {
                    intakeLeft.setPosition(0.522);
                    intakeRight.setPosition(0.52);
                } else {
                    intakeLeft.setPosition(0);
                    intakeRight.setPosition(0);
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

                if (gamepad2.x && intake_to_outtake != Intake_To_OutTake.START) {
                    intake_to_outtake = Intake_To_OutTake.START;
                }

                //OUTTAKE LEFT LOCK OPEN
                if (gamepad2.left_bumper) {
                    lockLeft.setPosition(0);
                }

                //OUTTAKE RIGHT LOCK OPEN
                if (gamepad2.right_bumper) {
                    lockRight.setPosition(0);
                }

                // DRONE LAUNCHER
                if (gamepad2.dpad_down) {
                    drone.setPosition(0.2);
                }
        }
    }

