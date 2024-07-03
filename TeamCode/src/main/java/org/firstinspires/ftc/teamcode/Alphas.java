package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous (name = "Alphas 19343 Blue")
public class Alphas extends LinearOpMode {

    private DcMotor pixelMotor = null;
    private DcMotor slideRight = null;
    private DcMotor slideLeft = null;
    private Servo intakeLeftServo = null;
    private Servo intakeRightServo = null;
    private Servo outtakeLeftServo = null;
    private Servo outtakeRightServo = null;

    private Servo lockFrontServo = null;

    private AprilTagFollowing apriltag = new AprilTagFollowing();


    public static String TEAM_NAME = "TEAM ALPHAS"; //TODO: Enter team Name
    public static int TEAM_NUMBER = 19343; //TODO: Enter team Number

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private static final String TFOD_MODEL_ASSET = "blue cube model.tflite";

    private static final String[] LABELS = {
            "cube",
    };

    //Vision parameters
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;

    public enum IDENTIFIED_SPIKE_MARK_LOCATION {
        LEFT,
        MIDDLE,
        RIGHT
    }
    public static IDENTIFIED_SPIKE_MARK_LOCATION identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;

    double slide_val = 0.3;
    @Override
    public void runOpMode() throws InterruptedException {

        //pixelMotor  = hardwareMap.get(DcMotor.class, "pixelMotor");
        //

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();
        telemetry.addData("Selected Starting Position", startPosition);

        //Activate Camera Vision that uses TensorFlow for cube detection
        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addLine("Vision Tensor Flow for Blue Cube Detection");
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
            telemetry.addLine("Vision Tensor Flow for Blue Cube Detection");
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
        Pose2d moveBeyondTrussPose = new Pose2d(0,0,0);
        Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
        Pose2d dropPurplePixelTruss = new Pose2d(0,0,0);
        Pose2d midwayPose1 = new Pose2d(0,0,0);
        Pose2d midwayPose1a = new Pose2d(0,0,0);
        Pose2d intakeStack = new Pose2d(0,0,0);
        Pose2d intakeStack2 = new Pose2d(0,0,0);
        Pose2d intakeStack3 = new Pose2d(0,0,0);
        Pose2d intakeStack4 = new Pose2d(0,0,0);
        Pose2d midwayPose2 = new Pose2d(0,0,0);
        Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0);
        Pose2d pixelAlignBlackboard = new Pose2d (0,0,0);
        Pose2d pixelDropSafetyPose = new Pose2d(0,0,0);
        Pose2d parkPose = new Pose2d(0,0, 0);
        double waitSecondsBeforeDrop = 0;
        org.firstinspires.ftc.teamcode.MecanumDrive drive = new org.firstinspires.ftc.teamcode.MecanumDrive(hardwareMap, initPose);

        initPose = new Pose2d(0, 0, Math.toRadians(0)); //Starting pose
        moveBeyondTrussPose = new Pose2d(1,0,0);

        switch (startPosition) {
            case BLUE_LEFT:
                drive = new org.firstinspires.ftc.teamcode.MecanumDrive(hardwareMap, initPose);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(21, 4.5, Math.toRadians(20));
                        dropPurplePixelTruss = new Pose2d(21, 4.5, Math.toRadians(20));
                        dropYellowPixelPose = new Pose2d(20, 40, Math.toRadians(90));
                        pixelAlignBlackboard = new Pose2d(20, 42, Math.toRadians(90));
                        pixelDropSafetyPose = new Pose2d(20, 36, Math.toRadians(90));
                        break;
                    // robot starting pose center
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(23.5, 0, Math.toRadians(0));
                        dropPurplePixelTruss = new Pose2d(23.5, 0, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(24.25, 41,  Math.toRadians(90));
                        pixelAlignBlackboard = new Pose2d(24.25, 43, Math.toRadians(90));
                        pixelDropSafetyPose = new Pose2d(24.25, 37, Math.toRadians(90));
                        break;
                    // robot starting pose center
                    case RIGHT:

                        dropPurplePixelPose = new Pose2d(27,0 , Math.toRadians(-90));
                        dropPurplePixelTruss = new Pose2d(27, -5.5, Math.toRadians(-90));
                        dropYellowPixelPose = new Pose2d(33, 40, Math.toRadians(85));
                        pixelAlignBlackboard = new Pose2d(33, 41, Math.toRadians(90));
                        pixelDropSafetyPose = new Pose2d(33, 36, Math.toRadians(90));
                        break;
                    // robot starting pose closer to the pins
                }

                midwayPose1 = new Pose2d(14, 13, Math.toRadians(45));
                waitSecondsBeforeDrop = 0.5; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(-3, 36, Math.toRadians(90));
                break;

            case RED_RIGHT:
                drive = new org.firstinspires.ftc.teamcode.MecanumDrive(hardwareMap, initPose);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(23, 5, Math.toRadians(35));
                        dropPurplePixelTruss = new Pose2d(23, 5, Math.toRadians(35));
                        dropYellowPixelPose = new Pose2d(33, -42, Math.toRadians(-90));
                        pixelAlignBlackboard = new Pose2d(33, -42, Math.toRadians(-90));
                        pixelDropSafetyPose = new Pose2d(33, -36, Math.toRadians(-90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(26.5, -3, Math.toRadians(0));
                        dropPurplePixelTruss = new Pose2d(26.5, -3, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(26.5, -42,  Math.toRadians(-90));
                        pixelAlignBlackboard = new Pose2d(26.5, -42, Math.toRadians(-90));
                        pixelDropSafetyPose = new Pose2d(26.5, -36, Math.toRadians(-90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(26, -16, Math.toRadians(0));
                        dropPurplePixelTruss = new Pose2d(26, -16, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(21, -42, Math.toRadians(-90));
                        pixelAlignBlackboard = new Pose2d(21, -42, Math.toRadians(-90));
                        pixelDropSafetyPose = new Pose2d(21, -36, Math.toRadians(-90));
                        break;
                }
                midwayPose1 = new Pose2d(14, -13, Math.toRadians(45));
                waitSecondsBeforeDrop = 1; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(2, -30, Math.toRadians(-90));
                break;

            case BLUE_RIGHT:
                drive = new org.firstinspires.ftc.teamcode.MecanumDrive(hardwareMap, initPose);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(24, 8, Math.toRadians(30));
                        dropPurplePixelTruss = new Pose2d(24, 8, Math.toRadians(30));
                        dropYellowPixelPose = new Pose2d(22, 41, Math.toRadians(90));
                        pixelAlignBlackboard = new Pose2d(22, 42, Math.toRadians(90));
                        pixelDropSafetyPose = new Pose2d(22, 36, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(24, 0, Math.toRadians(0));
                        dropPurplePixelTruss = new Pose2d(24, 0, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(18, 87,  Math.toRadians(90));
                        pixelAlignBlackboard = new Pose2d(18, 88, Math.toRadians(90));
                        pixelDropSafetyPose = new Pose2d(18, 80, Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(26, -7, Math.toRadians(-70));
                        dropPurplePixelTruss = new Pose2d(26, -7, Math.toRadians(-70));
                        dropYellowPixelPose = new Pose2d(24, 87, Math.toRadians(90));
                        pixelAlignBlackboard = new Pose2d(24, 88, Math.toRadians(90));
                        pixelDropSafetyPose = new Pose2d(24, 81, Math.toRadians(90));
                        break;
                }
                midwayPose1 = new Pose2d(8, -8, Math.toRadians(0));
                //midwayPose1a = new Pose2d(8, -10, Math.toRadians(0));
                //safeWaitSeconds(1);
                intakeStack = new Pose2d(54, -20,Math.toRadians(95));
                intakeStack2 = new Pose2d(50, -8 ,Math.toRadians(90));
                intakeStack3 = new Pose2d(46,-20,Math.toRadians(90));
                intakeStack4 = new Pose2d(46,-16,Math.toRadians(90));
                midwayPose2 = new Pose2d(50, 62, Math.toRadians(90));
                waitSecondsBeforeDrop = 0.5; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(44, 84, Math.toRadians(90));
                break;

            case RED_LEFT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(26, 8, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(40, -86, Math.toRadians(90));
                        pixelAlignBlackboard = new Pose2d(40, -85, Math.toRadians(90));
                        pixelDropSafetyPose = new Pose2d(40, -79, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(30, -3, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(29, -86, Math.toRadians(90));
                        pixelAlignBlackboard = new Pose2d(29, -85, Math.toRadians(90));
                        pixelDropSafetyPose = new Pose2d(29, -79, Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(28, -7, Math.toRadians(-35));
                        dropYellowPixelPose = new Pose2d(23, -86, Math.toRadians(90));
                        pixelAlignBlackboard = new Pose2d(23, -85, Math.toRadians(90));
                        pixelDropSafetyPose = new Pose2d(23, -79, Math.toRadians(90));
                        break;
                }
                midwayPose1 = new Pose2d(8, 8, Math.toRadians(0));
                midwayPose1a = new Pose2d(18, 15, Math.toRadians(90));
                intakeStack = new Pose2d(52, 15,Math.toRadians(95));
                midwayPose2 = new Pose2d(52, -62, Math.toRadians(90));
                waitSecondsBeforeDrop = 0.5; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(50, -84, Math.toRadians(90));
                break;
        }

        //Move robot to dropPurplePixel based on identified Spike Mark Location
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveBeyondTrussPose.position, moveBeyondTrussPose.heading)
                        .strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                        .build());

        //Code to drop Purple Pixel on Spike Mark
        safeWaitSeconds(0.5);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(dropPurplePixelTruss.position, dropPurplePixelTruss.heading)
                        .build());

        //Move robot to midwayPose1
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());

        //For Blue Right and Red Left, intake pixel from stack
        if (startPosition == START_POSITION.BLUE_RIGHT ||
                startPosition == START_POSITION.RED_LEFT) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            //.strafeToLinearHeading(midwayPose1a.position, midwayPose1a.heading)
                            .strafeToLinearHeading(intakeStack.position, intakeStack.heading)
                            .build());

            //TODO : Code to intake pixel from stack
            safeWaitSeconds(1);
            intakeLeftServo.setPosition(0.13);
            intakeRightServo.setPosition(0.87);
            pixelMotor.setPower(1);

            safeWaitSeconds(1);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(intakeStack2.position, intakeStack2.heading)
                            .build());

            intakeLeftServo.setPosition(0.1);
            intakeRightServo.setPosition(0.9);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(intakeStack3.position, intakeStack3.heading)
                            .build());
            safeWaitSeconds(1);
            pixelMotor.setPower(0);
            safeWaitSeconds(1);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(intakeStack4.position, intakeStack4.heading)
                            .build());
            //Move robot to midwayPose2 and to dropYellowPixelPose
            intakeLeftServo.setPosition(0.15);
            intakeRightServo.setPosition(0.85);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                            .build());
        }


        safeWaitSeconds(waitSecondsBeforeDrop);

        //Move robot to midwayPose2 and to dropYellowPixelPose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToLinearHeading(dropYellowPixelPose,0)
                        .build());


        //TODO : Code to drop Pixel on Backdrop
        safeWaitSeconds(1);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(dropYellowPixelPose.position, dropYellowPixelPose.heading)
                        .build());
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(pixelAlignBlackboard.position, dropYellowPixelPose.heading)
                        .build());
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the encoder to 0 position
        slideRight.setTargetPosition(-250); // Each 385 ticks is one rotation
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Assign motor to run to position
        slideRight.setPower(slide_val);// At 30% power
        slideLeft.setPower(slide_val);
        lockFrontServo.setPosition(0.07);
        //while (slideRight.isBusy()) {
        // While the motor is still making its way to 385, do nothing
        safeWaitSeconds(0.5);
        //outtakeRightServo.setPosition(1 - 0.53);
        //sleep(1000);
        //lockFrontServo.setPosition(0.27);
        //sleep(1000);
        outtakeRightServo.setPosition(1-0.575);
        outtakeLeftServo.setPosition(0.575);
        safeWaitSeconds(1);
        lockFrontServo.setPosition(0.27);
        safeWaitSeconds(0.5);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToLinearHeading(pixelDropSafetyPose,0)
                        .build());

        safeWaitSeconds(1);
        outtakeRightServo.setPosition(1-0.02);
        outtakeLeftServo.setPosition(0.02);
        slideLeft.setPower(0);
        slideRight.setPower(0);


        //}


        //Move robot to park in Backstage
        safeWaitSeconds(0.5);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
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
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:","");
            telemetry.addData("    Blue Left   ", "(X / ▢)");
            telemetry.addData("    Blue Right ", "(Y / Δ)");
            telemetry.addData("    Red Left    ", "(B / O)");
            telemetry.addData("    Red Right  ", "(A / X)");
            if(gamepad1.x){
                startPosition = START_POSITION.BLUE_LEFT;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.BLUE_RIGHT;
                break;
            }
            if(gamepad1.b){
                startPosition = START_POSITION.RED_LEFT;
                break;
            }
            if(gamepad1.a){
                startPosition = START_POSITION.RED_RIGHT;
                break;
            }
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

            if (startPosition == START_POSITION.RED_LEFT || startPosition == START_POSITION.BLUE_LEFT) {
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

    }   // end method runTfodTensorFlow()

}   // end class
