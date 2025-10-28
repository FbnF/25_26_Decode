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

@TeleOp(name = "TeleOpMain", group = "DecodeTeleopAllInOne")
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

    // --- Shooter manual/auto mode ---
    private boolean manualMode = false; // false = AUTO (AprilTags), true = MANUAL mode
    private double manualTps = 1800;    // starting target in manual mode
    private static final double MAN_TPS_MIN  = 800;
    private static final double MAN_TPS_MAX  = 3000;
    private static final double MAN_TPS_STEP = 50;

    // --- Button edge detection ---
    private boolean prevRB = false, prevDpadRight = false, prevDpadLeft = false, prevA = false;
    private boolean prevX = false, prevUp = false, prevDown = false, prevB = false, prevY = false, prevLB_GP2 = false;

    private long lastFeedNs = 0; // cooldown for feed toggle


    private boolean feedPulseActive = false;
    private long feedPulseStartNs = 0;
    private static final long FEED_DWELL_NS = 150_000_000L; // 150 ms

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

        // Drive (verify your constructor signature)
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Vision
        tagService = new AprilTagService();
        tagService.start(hardwareMap);

        waitForStart();

        // Safe startup
        intakeMotor.setPower(0.0);
        launchMotor.setPower(0.0);

        while (opModeIsActive()) {

            // --- Drive ---
            boolean driverbEdge = gamepad1.right_bumper && !drivePrevRB;
            boolean drivelbEdge = gamepad1.left_bumper  && !drivePrevLB;
            if (driverbEdge) speedFactor = Math.min(SPEED_MAX, speedFactor + SPEED_STEP);
            if (drivelbEdge) speedFactor = Math.max(SPEED_MIN, speedFactor - SPEED_STEP);

            double axial   = -gamepad1.right_stick_y * speedFactor; // up = forward (+x)
            double lateral = -gamepad1.left_stick_x  * speedFactor; // right = strafe right (−y)
            double heading = -gamepad1.right_stick_x * speedFactor; // right = turn right (−CCW = CW)

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(axial, lateral), heading));
            drive.setDrivePowers(
                    new PoseVelocity2d(new Vector2d(axial, lateral), heading)
            );

            telemetry.addData("Speed Factor", "%.2f (%.0f%%)", speedFactor, speedFactor*100);


            // --- Vision toggle (Left Bumper on gamepad2) ---
            /*
            boolean lbEdge = gamepad2.left_bumper && !prevLB_GP2;
            if (lbEdge) {
                visionEnabled = !visionEnabled;
                if (visionEnabled) {
                    tagService.start(hardwareMap);
                } else {
                    tagService.stop();
                }
            }
*/
            // --- AprilTag reading (smoothed inches) ---
            double rangeIn = Double.NaN;
            AprilTagService.Reading reading = null;

            if (visionEnabled) {
                reading = tagService.getLatest();
                if (reading == null || !reading.hasTag ) { // If Tag not detected
                    telemetry.addLine("🟥 AprilTag: NOT DETECTED");
                } else {
                    telemetry.addLine("🟩 AprilTag: DETECTED");
                    telemetry.addData("Position (in)", String.format("X: %.1f  Y: %.1f  Z: %.1f",
                            reading.xIn, reading.yIn, reading.zIn));
                    rangeIn = reading.smoothedDistanceIn; // may be NaN if we haven’t seen a tag yet
                }
            } else {
                telemetry.addLine("📷 Vision: OFF");
            }

            /* -------------------- COMMENTED OUT: Manual/AUTO mode & manual TPS control (avoids X/B/Y clashes) --------------------
            // --- Manual/AUTO mode toggle & manual TPS control ---
            boolean xEdge    = gamepad2.x && !prevX;      // toggle manual/auto
            boolean upEdge   = gamepad2.dpad_up && !prevUp;   // increase TPS
            boolean downEdge = gamepad2.dpad_down && !prevDown; // decrease TPS
            boolean bEdge    = gamepad2.b && !prevB;      // preset 1
            boolean yEdge    = gamepad2.y && !prevY;      // preset 2

            if (xEdge) manualMode = !manualMode; // toggle mode

            if (manualMode) {
                if (upEdge)   manualTps = Math.min(MAN_TPS_MAX, manualTps + MAN_TPS_STEP);
                if (downEdge) manualTps = Math.max(MAN_TPS_MIN, manualTps - MAN_TPS_STEP);

                // Optional: analog fine-tuning with triggers
                double lt = gamepad2.left_trigger;
                double rt = gamepad2.right_trigger;
                double trim = (rt - lt) * 200; // adjust 200 tps per full trigger press
                manualTps = clamp(manualTps + trim, MAN_TPS_MIN, MAN_TPS_MAX);

                // Presets (optional)
                if (bEdge) manualTps = clamp(1600, MAN_TPS_MIN, MAN_TPS_MAX);
                if (yEdge) manualTps = clamp(2400, MAN_TPS_MIN, MAN_TPS_MAX);
            }
            -------------------- END COMMENTED OUT -------------------- */

            // --- Shooter control using config + calculations ---
            Double tpsTarget = null;
/*
            if (manualMode) {
                tpsTarget = manualTps;
            } else if (!Double.isNaN(rangeIn) && rangeIn > ShooterConfig.MIN_RANGE_IN) {
                double tps = Calculations.computeTPSFromRangeInches(
                        ShooterConfig.G,
                        rangeIn,
                        ShooterConfig.LAUNCH_DEG,
                        ShooterConfig.SHOOTER_H_M,
                        ShooterConfig.TARGET_H_M,
                        ShooterConfig.WHEEL_RADIUS_M,
                        ShooterConfig.EFFICIENCY,
                        ShooterConfig.TICKS_PER_REV
                );

                if (!Double.isNaN(tps)) {
                    tpsTarget = tps;
                } else {
                    launchMotor.setVelocity(0.0);
                    telemetry.addLine("Shooter TPS invalid → motor stopped");
                }
            }
*/
            // --- Four fixed power levels + feed pulse trigger ---
            if (gamepad2.a){
                launchPower = 0.65;
            }
            if (gamepad2.b){
                launchPower = 0.78;
            }
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
            telemetry.addData("Launch Motor Speed", launchPower);
            launchMotor.setPower(launchPower);


            if (feedPulseActive) {
                long now = System.nanoTime();
                if (now - feedPulseStartNs >= FEED_DWELL_NS) {
                    feedServo.setPosition(0.0);
                    feedPulseActive = false;
                }
            }

            if (tpsTarget != null) {
                // --- Feed servo toggle (A edge) ---
                boolean aEdge = gamepad2.a && !prevA; // Detects the moment the A is newly pressed (rising edge)
                launchMotor.setVelocity(tpsTarget);
                // read once
                double vel = launchMotor.getVelocity();
                boolean speedOk = Math.abs(vel - tpsTarget) <= ShooterConfig.TPS_TOL;
                long now = System.nanoTime();
                boolean cooldownOk = (now - lastFeedNs) > 150_000_000L; // 150ms

                if (speedOk) {
                    if (aEdge && cooldownOk) {
                        isFeedServoDown = !isFeedServoDown; // Sets isFeeServoDown flag to the opposite of what it was
                        double pos = isFeedServoDown ? 1.0 : 0.0;
                        feedServo.setPosition(Math.max(0.0, Math.min(1.0, pos))); // clamp to [0,1]
                        lastFeedNs = now;
                    }
                    telemetry.addLine("Shooter READY");

                } else telemetry.addLine("Shooter hasn't reached correct speed");
                telemetry.addData("Shooter Mode", manualMode ? "MANUAL" : "AUTO");
                telemetry.addData("TPS Target", tpsTarget);
                telemetry.addData("TPS Measured", vel);
                telemetry.addData("Δ TPS", "%.1f", tpsTarget - vel);
                telemetry.addData("Tol (≤)", "%.1f", ShooterConfig.TPS_TOL);
            }
            // (Keep the previously removed else that forced setVelocity(0.0) removed.)

            // --- Intake toggle (RB edge) ---

            boolean rbEdge = gamepad2.right_bumper && !prevRB; // rising edge
            if (rbEdge) {
                isIntakeRunning = !isIntakeRunning;
                if (isIntakeRunning) {
                    if (intakePower <= 0.0) {
                        intakePower = 1.0; // default start power
                    }

                } else {
                    intakePower = 0.0;
                }
                intakeMotor.setPower(intakePower);
            }





            // --- Intake power trim with dpad (edges) ---
            boolean dpadRightEdge = gamepad2.dpad_right && !prevDpadRight; // rising edge
            boolean dpadLeftEdge  = gamepad2.dpad_left  && !prevDpadLeft;  // rising edge
            if (dpadRightEdge)      intakePower = Math.min(1.0,  intakePower + 0.1);
            else if (dpadLeftEdge)  intakePower = Math.max(0.0, intakePower - 0.1);
            if (isIntakeRunning)    intakeMotor.setPower(intakePower);

            telemetry.addData("Vision", visionEnabled ? "ON" : "OFF");
            telemetry.addData("Intake", isIntakeRunning ? "RUNNING" : "STOPPED");
            telemetry.addData("Intake Power", "%.1f", intakePower);
            telemetry.addData("Shooter Vel (tps)", "%.1f", launchMotor.getVelocity());
            telemetry.update();

            // Edge bookkeeping
            prevRB = gamepad2.right_bumper;
            prevDpadRight = gamepad2.dpad_right;
            prevDpadLeft = gamepad2.dpad_left;
            prevA = gamepad2.a;
            prevX = gamepad2.x;
            prevUp = gamepad2.dpad_up;
            prevDown = gamepad2.dpad_down;
            prevB = gamepad2.b;
            prevY = gamepad2.y;
            prevLB_GP2 = gamepad2.left_bumper;

            // end-of-loop edge bookkeeping
            drivePrevRB = gamepad1.right_bumper;
            drivePrevLB = gamepad1.left_bumper;
        }
        // ---- cleanup runs after STOP is pressed ----
        try {
            if (launchMotor != null) launchMotor.setVelocity(0.0);
            if (intakeMotor != null) intakeMotor.setPower(0.0);
        } finally {
            // Make sure the camera is freed so the next OpMode can open it
            if (tagService != null) tagService.stop();
        }

    }


    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}