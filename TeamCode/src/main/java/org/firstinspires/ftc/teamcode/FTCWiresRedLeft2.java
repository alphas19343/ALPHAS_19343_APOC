
package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/**
 * FTC WIRES Autonomous Example for only vision detection using tensorflow and park
 */
@Autonomous(name = "FTC Wires RedLeft2", group = "00-Autonomous", preselectTeleOp = "FTC Wires TeleOp")
public class FTCWiresRedLeft2 extends LinearOpMode {
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
    public static String TEAM_NAME = "Alphas"; //TODO: Enter team Name
    public static int TEAM_NUMBER = 19343; //TODO: Enter team Number

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    //Vision parameters
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    private static final String TFOD_MODEL_ASSET = "red cube model.tflite";

    private static final String[] LABELS = {
            "cube",
    }
            ;

    //Define and declare Robot Starting Locations
    public enum START_POSITION{

        RED_LEFT


    }
    public static START_POSITION startPosition;

    public enum IDENTIFIED_SPIKE_MARK_LOCATION {
        LEFT,
        MIDDLE,
        RIGHT
    }
    public static IDENTIFIED_SPIKE_MARK_LOCATION identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;

    @Override
    public void runOpMode() throws InterruptedException {

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

        lockRight.setPosition(0.55);
        //lockLeft.setPosition(0.55);

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();
        telemetry.addData("Selected Starting Position", startPosition);

        //Activate Camera Vision that uses TensorFlow for pixel detection
        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addLine("Vision Tensor Flow for White Pixel Detection");
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addLine("The starting point of the robot is assumed to be on the starting tile, " +
                "and along the edge farther from the truss legs. ");
        telemetry.addLine("You should also have a webcam connected and positioned in a way to see " +
                "the middle spike mark and the spike mark away from the truss (and ideally nothing else). " +
                "We assumed the camera to be in the center of the robot. ");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        //waitForStart();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Selected Starting Position", startPosition);

            //Run Vuforia Tensor Flow and keep watching for the White Pixel on the spike mark.
            runTfodTensorFlow();
            telemetry.addLine("Vision Tensor Flow for White Pixel Detection");
            telemetry.addData("Identified Parking Location", identifiedSpikeMarkLocation);
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            runAutonoumousMode();
        }
    }   // end runOpMode()

    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d moveBeyondTrussPose = new Pose2d(0, 0, 0);
        Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
        Pose2d midwayPose1 = new Pose2d(0, 0, 0);
        Pose2d midwayPose1a = new Pose2d(0, 0, 0);
        Pose2d intakeStack = new Pose2d(0, 0, 0);
        Pose2d midwayPose2 = new Pose2d(0, 0, 0);
        Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0);
        Pose2d parkPose = new Pose2d(0, 0, 0);
        Pose2d midwayPose3 = new Pose2d(0,0,0);
        double waitSecondsBeforeDrop = 0;
        org.firstinspires.ftc.teamcode.MecanumDrive drive = new org.firstinspires.ftc.teamcode.MecanumDrive(hardwareMap, initPose);

        initPose = new Pose2d(0, 0, Math.toRadians(0)); //Starting pose
        moveBeyondTrussPose = new Pose2d(15, 0, 0);


        Pose2d dropPurplePixelPose_2 = null;
        Pose2d parkPose_2 = null;
        Pose2d pixelAlignBlackboard = null;
        Pose2d pixelDropSafetyPose = null;
        Pose2d intakeStack_2 = null;
        Pose2d intakeStack_3 = null;
        Pose2d moveBeyondTrussPose_2 = null;
        Pose2d intakeStack_pre = null;
        switch (startPosition) {
            case RED_LEFT:
                drive = new org.firstinspires.ftc.teamcode.MecanumDrive(hardwareMap, initPose);
                switch (identifiedSpikeMarkLocation) {
                    case LEFT:
                        //dropPurplePixelPose = new Pose2d(24, 8, Math.toRadians(30));
                        moveBeyondTrussPose_2 = new Pose2d(20, 0, 0);
                        dropPurplePixelPose = new Pose2d(25, -15, Math.toRadians(87));

                        midwayPose1 = new Pose2d(32, -8, Math.toRadians(90));
                        midwayPose1a = new Pose2d(37, -17, Math.toRadians(90));
                        intakeStack_pre = new Pose2d(48, -17, Math.toRadians(88.5));
                        intakeStack = new Pose2d(48, -25.4, Math.toRadians(90));

                        intakeStack_3 = new Pose2d(47.5, -20, Math.toRadians(87));

                        midwayPose2 = new Pose2d(52, 80, Math.toRadians(90));
                        waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                        parkPose_2 = new Pose2d(34, 78, Math.toRadians(90));
                        parkPose = new Pose2d(50, 84, Math.toRadians(0));

                        dropPurplePixelPose_2 = new Pose2d(26, 0.655, Math.toRadians(89.5));
                        dropYellowPixelPose = new Pose2d(27, 85, Math.toRadians(90));
                        pixelAlignBlackboard = new Pose2d(26, 83, Math.toRadians(90));
                        pixelDropSafetyPose = new Pose2d(25, 88, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(29, 0, Math.toRadians(0));
                        dropPurplePixelPose_2 = new Pose2d(22.2, 0, Math.toRadians(0));
                        midwayPose1 = new Pose2d(8, 8, Math.toRadians(-90));
                        midwayPose1a = new Pose2d(18, 17, Math.toRadians(-90));
                        intakeStack = new Pose2d(45, 25, Math.toRadians(-93));
                        intakeStack_2 = new Pose2d(47, 0, Math.toRadians(-87));
                        intakeStack_3 = new Pose2d(47, -20, Math.toRadians(-87));

                        midwayPose2 = new Pose2d(55, -10, Math.toRadians(-91.5));
                        waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                        parkPose_2 = new Pose2d(55, -78, Math.toRadians(-90));
                        parkPose = new Pose2d(55, -84, Math.toRadians(0));

                        dropYellowPixelPose = new Pose2d(38, -85, Math.toRadians(-90));
                        pixelAlignBlackboard = new Pose2d(37, -83, Math.toRadians(-90));
                        pixelDropSafetyPose = new Pose2d(36.5, - 88, Math.toRadians(-90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(25, -15, Math.toRadians(-87));
                        dropPurplePixelPose_2 = new Pose2d(28, 2.05, Math.toRadians(-92));
                        dropYellowPixelPose = new Pose2d(40, 87, Math.toRadians(90));
                        pixelAlignBlackboard = new Pose2d(37, 83, Math.toRadians(90));
                        pixelDropSafetyPose = new Pose2d(36.5, 88, Math.toRadians(90));

                        moveBeyondTrussPose_2 = new Pose2d(20, 0, 0);


                        midwayPose1 = new Pose2d(34, 1.5, Math.toRadians(-92));
                        midwayPose3 = new Pose2d(28, 2.05, Math.toRadians(0));
                        midwayPose1a = new Pose2d(40, 2.05, Math.toRadians(90));
                        intakeStack_pre = new Pose2d(48, -17, Math.toRadians(88.5));
                        intakeStack = new Pose2d(48, -25.1, Math.toRadians(90));
                        intakeStack_2 = new Pose2d(47.5, 0, Math.toRadians(87));
                        intakeStack_3 = new Pose2d(47.5, -20, Math.toRadians(87));

                        midwayPose2 = new Pose2d(52, 80, Math.toRadians(90));
                        waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                        parkPose_2 = new Pose2d(34, 78, Math.toRadians(90));
                        parkPose = new Pose2d(50, 84, Math.toRadians(0));

                        break;
                    // robot starting pose closer to the pins
                }


                break;

        }

        //Move robot to dropPurplePixel based on identified Spike Mark Location

        if (identifiedSpikeMarkLocation == IDENTIFIED_SPIKE_MARK_LOCATION.LEFT || identifiedSpikeMarkLocation == IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(moveBeyondTrussPose.position, moveBeyondTrussPose.heading)
                            .strafeToLinearHeading(moveBeyondTrussPose_2.position, moveBeyondTrussPose_2.heading)

                            .build());

            safeWaitSeconds(0.5);

            LSSLeft.setPosition(0.5);
            LSSRight.setPosition(0.5);

            safeWaitSeconds(0.5);

            LSSLeft.setPosition(0);
            LSSRight.setPosition(0);

            safeWaitSeconds(0.5);

            arm.setPosition(0);
            rotate.setPosition(0.45);

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                            .strafeToLinearHeading(dropPurplePixelPose_2.position, dropPurplePixelPose_2.heading)

                            .build());

            lockRight.setPosition(0);


        }

        if (identifiedSpikeMarkLocation == IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(moveBeyondTrussPose.position, moveBeyondTrussPose.heading)
                            //-.strafeToLinearHeading(moveBeyondTrussPose_2.position, moveBeyondTrussPose_2.heading)
                            .strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                            .strafeToLinearHeading(dropPurplePixelPose_2.position, dropPurplePixelPose_2.heading)

                            .build());
            safeWaitSeconds(0.5);

            LSSLeft.setPosition(0.5);
            LSSRight.setPosition(0.5);

            safeWaitSeconds(0.5);

            LSSLeft.setPosition(0);
            LSSRight.setPosition(0);

            safeWaitSeconds(0.5);

            arm.setPosition(0);
            rotate.setPosition(0.45);

            safeWaitSeconds(1);
            lockRight.setPosition(0);


        }

        if (identifiedSpikeMarkLocation == IDENTIFIED_SPIKE_MARK_LOCATION.LEFT){
            safeWaitSeconds(0.5);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                            .build());

            LSSLeft.setPosition(0.85);
            LSSRight.setPosition(0.85);
            arm.setPosition(0.78);
            rotate.setPosition(0.45);
        }
        //Code to drop Purple Pixel on Spike Mark

        if (identifiedSpikeMarkLocation == IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT){
            safeWaitSeconds(0.5);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose3.position, midwayPose3.heading)
                            .build());

            LSSLeft.setPosition(0.85);
            LSSRight.setPosition(0.85);
            arm.setPosition(0.78);
            rotate.setPosition(0.45);

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1a.position, midwayPose3.heading)
                            .build());


        }

        if (identifiedSpikeMarkLocation == IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE){
            safeWaitSeconds(0.5);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                            .build());

            LSSLeft.setPosition(0.85);
            LSSRight.setPosition(0.85);
            arm.setPosition(0.78);
            rotate.setPosition(0.45);
        }



        if (identifiedSpikeMarkLocation == IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE){

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1a.position, midwayPose1a.heading)

                            .build());

        }

        //Move robot to midwayPose1
        if (identifiedSpikeMarkLocation == IDENTIFIED_SPIKE_MARK_LOCATION.LEFT || identifiedSpikeMarkLocation == IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            //.strafeToLinearHeading(midwayPose1a.position, midwayPose1a.heading)
                            .strafeToLinearHeading(intakeStack_pre.position, intakeStack_pre.heading)

                            .build());
        }




        //For Blue Right and Red Left, intake pixel from stack
        safeWaitSeconds(0.5);



        if (identifiedSpikeMarkLocation == IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE){

            intakeLeft.setPosition(0.32);
            intakeRight.setPosition(0.32);

        }

        if (identifiedSpikeMarkLocation == IDENTIFIED_SPIKE_MARK_LOCATION.LEFT || identifiedSpikeMarkLocation == IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT || identifiedSpikeMarkLocation == IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE){
            rollers.setPower(-0.7);
            ramp.setPower(1);
            safeWaitSeconds(1);

            intakeLeft.setPosition(0.36);
            intakeRight.setPosition(0.36);
        }

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(intakeStack.position, intakeStack.heading)

                        .build());


        rollers.setPower(-0.7);
        ramp.setPower(1);
        safeWaitSeconds(1);

//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(intakeStack_2.position, intakeStack_2.heading)
//                        .build());

//        LSSLeft.setPosition(0.94);
//        LSSRight.setPosition(0.94);
//        arm.setPosition(0.85);
//
//        intakeLeft.setPosition(0.522);
//        intakeRight.setPosition(0.52);
//
//
//
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(intakeStack_3.position, intakeStack_3.heading)
//                        .build());
//
//        rollers.setPower(-0.7);
//        ramp.setPower(1);
//        safeWaitSeconds(1);


        //Move robot to midwayPose2 and to dropYellowPixelPose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                        .build());
        intakeLeft.setPosition(0);
        intakeRight.setPosition(0);
        rollers.setPower(0);
        ramp.setPower(0);

        safeWaitSeconds(0.5);

        LSSLeft.setPosition(0.94);
        LSSRight.setPosition(0.94);
        arm.setPosition(0.78);

        safeWaitSeconds(0.5);

        arm.setPosition(0.76);

        safeWaitSeconds(0.5);
        lockRight.setPosition(0.55);
        lockLeft.setPosition(0.55);

        //Move robot to midwayPose2 and to dropYellowPixelPose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToLinearHeading(dropYellowPixelPose, 0)
                        .strafeToLinearHeading(pixelAlignBlackboard.position, pixelAlignBlackboard.heading)
                        .build());


        //TODO : Code to drop Pixel on Backdrop

        safeWaitSeconds(0.5);

        LSSLeft.setPosition(0.5);
        LSSRight.setPosition(0.5);

        safeWaitSeconds(0.5);

        LSSLeft.setPosition(0);
        LSSRight.setPosition(0);

        safeWaitSeconds(0.5);

        arm.setPosition(0.12);
        rotate.setPosition(0.45);
        safeWaitSeconds(1);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(pixelDropSafetyPose.position, pixelDropSafetyPose.heading)
                        .build());

        rotate.setPosition(0.98);
        lockRight.setPosition(0);
        lockLeft.setPosition(0);


        safeWaitSeconds(0.5);


        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPose_2.position, parkPose_2.heading)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        //.splineToLinearHeading(parkPose,0)
                        .build());


    }


    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous adopted for Team:",
                    TEAM_NAME, " ", TEAM_NUMBER);
            telemetry.addData("---------------------------------------","");
            telemetry.addLine("This Auto program uses Vision Tensor Flow for White pixel detection");


            int x = 0;
            if (x == 0){startPosition = START_POSITION.RED_LEFT;
                break;}




            telemetry.update();
        }
        telemetry.clearAll();
    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();


        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.60f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void runTfodTensorFlow() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        //Camera placed between Left and Right Spike Mark on RED_LEFT and BLUE_LEFT If pixel not visible, assume Right spike Mark
        //if (startPosition == START_POSITION.RED_LEFT || startPosition == START_POSITION.BLUE_LEFT) {
        //  identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
        //} else { //RED_RIGHT or BLUE_RIGHT
        //  identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
        //
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            if (startPosition == START_POSITION.RED_LEFT ) {
                if (recognition.getLabel() == "cube") {
                    if (x < 150) {
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
                    } else if (x > 150 && x<400){
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                    }
                    else {
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
                    }
                }
            } else { //RED_RIGHT or BLUE_RIGHT
                if (recognition.getLabel() == "cube") {
                    if (x < 150) {
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
                    } else if (x > 150 && x<400){
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                    }
                    else {
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
                    }
                }
            }

        }   // end for() loop

    }    // end method runTfodTensorFlow()

}   // end class
