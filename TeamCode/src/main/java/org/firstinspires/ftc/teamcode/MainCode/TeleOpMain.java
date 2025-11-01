package org.firstinspires.ftc.teamcode.MainCode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive; // <-- adjust path if needed
import org.firstinspires.ftc.teamcode.MainCode.util.Calculations;
import org.firstinspires.ftc.teamcode.MainCode.config.ShooterConfig;
//import org.firstinspires.ftc.teamcode.MainCode.config.TagConfig;
import org.firstinspires.ftc.teamcode.MainCode.vision.AprilTagService;

// --- Data Logging ---
import org.firstinspires.ftc.teamcode.MainCode.util.TinyCsvLogger;
import com.acmerobotics.roadrunner.Pose2d;

@TeleOp(name = "TeleOp: Main", group = "TeleOp")
public class TeleOpMain extends LinearOpMode {

    // --- Hardware ---
    private Servo feedServo;
    private MecanumDrive drive;
    private DcMotorEx intakeMotor;
    private DcMotorEx launchMotor;

    // --- Vision ---
    private AprilTagService tagService;
    private boolean visionEnabled = true; // allows camera to be toggled on/off

    // --- Drive/settings ---
    private double speedFactor = 0.7;
    final double SPEED_MIN = 0.2;
    final double SPEED_MAX = 1.0;
    final double SPEED_STEP = 0.1;
    boolean drivePrevRB = false, drivePrevLB = false;

    // --- Intake/servo state ---
    private double intakePower = 0.0;
    private static double launchPower;
    private boolean isIntakeRunning = false;
    private boolean isLaunchRunning = false;
    private boolean isFeedServoDown = false;

   // --- Button edge detection ---
    private boolean prevRB = false;

    private boolean feedPulseActive = false;
    private long feedPulseStartNs = 0;
    private static final long FEED_DWELL_NS = 150_000_000L; // 150 ms

    private TinyCsvLogger logger; // LOG


    @Override
    public void runOpMode() {

        // Map hardware
        feedServo   = hardwareMap.get(Servo.class,    "feedServo");
        intakeMotor = hardwareMap.get(DcMotorEx.class,"IntakeMotor");
        launchMotor = hardwareMap.get(DcMotorEx.class,"LaunchMotor");

        feedServo.setPosition(0.0);
        isFeedServoDown = false;

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Drive (verify your constructor signature)
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Vision
        tagService = new AprilTagService();
        tagService.start(hardwareMap);

        // LOG: create CSV logger
        logger = TinyCsvLogger.create(hardwareMap, "teleop_main");


        waitForStart();

        // Safe startup
        intakeMotor.setPower(0.0);
        launchMotor.setPower(0.0);

        while (opModeIsActive()) {

            // -------------------------------- Base Drive -----------------------------------------
            if (gamepad1.a) speedFactor = 0.95;
            if (gamepad1.b) speedFactor = 0.4;
            if (gamepad1.x) speedFactor = 0.7;

            double axial   = -gamepad1.right_stick_y * speedFactor; // up = forward (+x)
            double lateral = -gamepad1.left_stick_x  * speedFactor; // right = strafe right (−y)
            double heading = -gamepad1.right_stick_x * speedFactor; // right = turn right (−CCW = CW)

            //drive.setDrivePowers(new PoseVelocity2d(new Vector2d(axial, lateral), heading));
            drive.setDrivePowers(
                    new PoseVelocity2d(new Vector2d(axial, lateral), heading)
            );

            telemetry.addData("Speed Factor", "%.2f (%.0f%%)", speedFactor, speedFactor*100);

            // --- Three fixed power levels + feed pulse trigger ---
            // Long range
            if (gamepad2.a){
                launchPower = 0.75;
            }
            // Middle range
            if (gamepad2.b){
                launchPower = 0.60;
            }
            // Short range
            if(gamepad2.left_bumper){
                launchPower=0.55;
            }
            // Turn off the LaunchMotor
            if (gamepad2.x){
                launchPower = 0;
            }
            if (gamepad2.y){
                if (!feedPulseActive && launchMotor.getPower() > 0.0) {
                    feedServo.setPosition(0.75);
                    feedPulseActive = true;
                    feedPulseStartNs = System.nanoTime();
                }
            }
            telemetry.addData("Launch Motor Power", launchPower);
            launchMotor.setPower(launchPower);


            if (feedPulseActive) {
                long now = System.nanoTime();
                if (now - feedPulseStartNs >= FEED_DWELL_NS) {
                    feedServo.setPosition(0.0);
                    feedPulseActive = false;
                }
            }
            // (Keep the previously removed else that forced setVelocity(0.0) removed.)

            // --- Intake toggle (RB edge) ---

            boolean rbEdge = gamepad2.right_bumper && !prevRB; // rising edge
            if (rbEdge) {intakePower=-0.5;}
            prevRB = gamepad2.right_bumper;
            if (gamepad2.right_trigger>0) {
                intakePower = 1.0; // default start power
            }
            if (gamepad2.left_trigger>0) {
                intakePower = 0.0;
            }

            intakeMotor.setPower(intakePower);

            // Get current estimated pose (position + heading)
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            logger.record(
                    "run",
                    launchPower,    // The commanded shooter power
                    launchMotor,    // The measured power + velocity
                    intakePower,    // The commanded intake power
                    intakeMotor,    // The measured power
                    feedServo,      // servo position
                    pose
            );

            // ------------- Telemetry data -------------------------------------------------
            telemetry.addData("Intake", isIntakeRunning ? "RUNNING" : "STOPPED");
            telemetry.addData("Intake Power", "%.1f", intakePower);
            telemetry.addData("Shooter Vel (tps)", "%.1f", launchMotor.getVelocity());
            telemetry.update();

        }
        // ---- cleanup runs after STOP is pressed ----
        try {
            if (launchMotor != null) launchMotor.setPower(0.0);
            if (intakeMotor != null) intakeMotor.setPower(0.0);
        } finally {
            // Make sure the camera is freed so the next OpMode can open it
            if (tagService != null) tagService.stop();
            if (logger != null) logger.close(); // LOG
        }

    }
}