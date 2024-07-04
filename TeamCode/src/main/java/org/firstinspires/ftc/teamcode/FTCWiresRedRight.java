
package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
@Autonomous(name = "FTC Wires RedRight", group = "00-Autonomous", preselectTeleOp = "FTC Wires TeleOp")
public class FTCWiresRedRight extends LinearOpMode {
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
    };

    //Define and declare Robot Starting Locations
    public enum START_POSITION{

        RED_RIGHT
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

        lockRight.setPosition(0.55);
        lockLeft.setPosition(0.55);

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
        Pose2d moveBeyondTrussPose = new Pose2d(0,0,0);
        Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
        Pose2d dropPurplePixelTruss_2 = new Pose2d(0,0,0);
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
        moveBeyondTrussPose = new Pose2d(20,0,0);

        Pose2d park_flip_pose = null;
        switch (startPosition) {
            case RED_RIGHT:
                drive = new org.firstinspires.ftc.teamcode.MecanumDrive(hardwareMap, initPose);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(29, 0, Math.toRadians(90));
                        dropPurplePixelTruss = new Pose2d(29, 5, Math.toRadians(90));
                        dropPurplePixelTruss_2 = new Pose2d(29, 0, Math.toRadians(90));
                        dropYellowPixelPose = new Pose2d(16, 39, Math.toRadians(90));
                        pixelAlignBlackboard = new Pose2d(16, 40, Math.toRadians(90));
                        pixelDropSafetyPose = new Pose2d(16, 37, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(29, 0, Math.toRadians(0));
                        dropPurplePixelTruss = new Pose2d(28.5, 0, Math.toRadians(0));
                        dropPurplePixelTruss_2 = new Pose2d(24.75, 0, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(21.505, 40, Math.toRadians(90));
                        pixelAlignBlackboard = new Pose2d(21.505, 41, Math.toRadians(90));
                        pixelDropSafetyPose = new Pose2d(21.505, 37, Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(26, 0, Math.toRadians(-90));
                        dropPurplePixelTruss = new Pose2d(26, -7.5, Math.toRadians(-90));
                        dropPurplePixelTruss_2 = new Pose2d(26, 3, Math.toRadians(-90));
                        dropYellowPixelPose = new Pose2d(29, 40, Math.toRadians(90));
                        pixelAlignBlackboard = new Pose2d(29, 41, Math.toRadians(90));
                        pixelDropSafetyPose = new Pose2d(29, 37, Math.toRadians(90));
                        break;
                    // robot starting pose closer to the pins
                }

                midwayPose1 = new Pose2d(14, 0, Math.toRadians(45));
                waitSecondsBeforeDrop = 0.5; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(-3, 36, Math.toRadians(90));
                park_flip_pose = new Pose2d(5, 36, Math.toRadians(4));
                break;

        }

        //Move robot to dropPurplePixel based on identified Spike Mark Location
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveBeyondTrussPose.position, moveBeyondTrussPose.heading)
                        .strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                        .build());

        //Code to drop Purple Pixel on Spike Mark
        //safeWaitSeconds(0.5);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(dropPurplePixelTruss.position, dropPurplePixelTruss.heading)
                        .strafeToLinearHeading(dropPurplePixelTruss_2.position, dropPurplePixelTruss_2.heading)
                        .build());


        LSSLeft.setPosition(0.5);
        LSSRight.setPosition(0.5);


        safeWaitSeconds(1);

        LSSLeft.setPosition(0);
        LSSRight.setPosition(0);

        safeWaitSeconds(1);

        arm.setPosition(0);
        rotate.setPosition(0.45);

        safeWaitSeconds(1);
        lockRight.setPosition(0);

        //Move robot to midwayPose1
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());

        //For Blue Right and Red Left, intake pixel from stack
        arm.setPosition(0.12);

        safeWaitSeconds(waitSecondsBeforeDrop);

        //Move robot to midwayPose2 and to dropYellowPixelPose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToLinearHeading(dropYellowPixelPose,0)
                        .build());


        //TODO : Code to drop Pixel on Backdrop
        safeWaitSeconds(0.1);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(dropYellowPixelPose.position, dropYellowPixelPose.heading)
                        .build());
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(pixelAlignBlackboard.position, dropYellowPixelPose.heading)
                        .build());


        safeWaitSeconds(0.20);

        lockLeft.setPosition(0);

        safeWaitSeconds(0.1);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToLinearHeading(pixelDropSafetyPose,0)
                        .build());

        safeWaitSeconds(0.1);

        //Move robot to park in Backstage
        safeWaitSeconds(0.1);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        //.splineToLinearHeading(parkPose,0)
                        .build());

        LSSLeft.setPosition(0.85);
        LSSRight.setPosition(0.85);
        arm.setPosition(0.85);
        rotate.setPosition(0.45);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(park_flip_pose.position, park_flip_pose.heading)
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
            if (x==0){
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

            if (startPosition == START_POSITION.RED_RIGHT ) {
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
