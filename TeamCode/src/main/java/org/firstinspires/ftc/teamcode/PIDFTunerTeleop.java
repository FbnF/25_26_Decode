package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PIDFTunerTeleop.DashTuning.manualTargetTPS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MainCode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.MainCode.util.TinyCsvLoggerFlex;
import org.firstinspires.ftc.teamcode.MainCode.vision.AprilTagService;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

//@Disabled
@TeleOp(name = "TeleOp: PIDFTune", group = "TeleOp")
public class PIDFTunerTeleop extends LinearOpMode {

    // ---------------- Dashboard Tunables ----------------
    @Config
    public static class DashTuning {
        // PIDF for RUN_USING_ENCODER velocity loop
        public static double P = 0.0;
        public static double I = 0.0;
        public static double D = 0.0;
        public static double F = 0.0;

        // Manual mode velocity command (ticks/sec)
        public static double manualTargetTPS = 0.0;

        // Clamp
        public static double manualMaxTPS = 2800.0;

        // Apply PIDF every loop (safe + simple)
        public static boolean applyPidfContinuously = true;

        // Manual "ready" tolerance (ticks/sec)
        public static double manualTolTPS = 80.0;
    }

    // --- Hardware ---
    private CRServo feedServo;
    private MecanumDrive drive;
    private DcMotorEx intakeMotor;
    private DcMotorEx launchMotor;
    private DcMotorEx launchMotor_2;
    private RevBlinkinLedDriver blinkin; // LED
    private VoltageSensor battery;
    private Limelight3A limelight;

    // --- Vision ---
    private AprilTagService tagService;
    private boolean visionEnabled = false; // allows camera to be toggled on/off

    // Auto shooter (closed-loop velocity) path
    private boolean autoShooter = false;
    private boolean prevDpadUp = false, prevDpadDown = false;
    private double shooterSetpointTPS = 0.0;
    private static final double NO_SETPOINT = 0.0;

    // --- Vision ---

    // Auto shooter (closed-loop velocity)
    double Tx;
    double Ty;

    // Debug: distance + pose
    double xM_dbg = 0, yM_dbg = 0, zM_dbg = 0;
    double xIn_dbg = 0, yIn_dbg = 0, zIn_dbg = 0;
    double rangeIn_dbg = 0;
    boolean hasGoalTag_dbg = false;

    // Additional debug: base TPS values
    private double distIn_raw_dbg = 0.0;
    private double distIn_filt_dbg = 0.0;
    private double physicsTps_dbg = 0.0;
    private double tableTps_dbg = 0.0;
    private double commandedBase_dbg = 0.0; // base TPS chosen BEFORE scale/offset (if any)
    private double finalTps_dbg = 0.0;// actual TPS commanded to motor

    private double lastTPS = 0.0;
    public static double FACTOR  = 0.8;
    private boolean noShotZone_dbg = false;

    // --- Config flags ---
    private static final boolean LOG_ENABLED = true;  // turn CSV logging on/off
    private TinyCsvLoggerFlex logger; // logging Data
    private static final int GOAL_TAG_ID = 20;        // 20 = blue goal, 24 = red goal

    // require driver to arm auto-spin before controlling flywheel
    private boolean autoSpinArmed = false;
    private boolean prevDpadRight = false;

    // flash window when Y pressed too soon
    private long yTooSoonFlashUntilNs = 0L;
    private static final long FLASH_YELLOW_NS = 500_000_000L; // 500 ms

    // --- Drive/settings ---
    private double speedFactor = 0.7;
    final double SPEED_MIN = 0.2;
    final double SPEED_MAX = 1.0;
    final double SPEED_STEP = 0.1;
    boolean drivePrevRB = false, drivePrevLB = false;

    // --- Intake/servo state ---
    private double intakePower = 0.0;
    private double launchPowerVel;
    private boolean isIntakeRunning = false;
    private boolean isLaunchRunning = false;
    private boolean isFeedServoDown = false;

    // --- Button edge detection ---
    private boolean prevRB = false;

    private boolean feedPulseActive = false;
    private long feedPulseStartNs = 0;
    private static final long FEED_DWELL_NS = 150_000_000L; // 150 ms

    //edge state for GP1 dpad-down (vision toggle)
    private boolean prevG1DpadDown = false;
    private double CompPower;

    // local tracking for "only apply when changed"
    private PIDFCoefficients lastAppliedPidf = null;

    @Override
    public void runOpMode() {

        // Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Map hardware
        feedServo = hardwareMap.get(CRServo.class, "feedServo");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        launchMotor = hardwareMap.get(DcMotorEx.class, "LaunchMotor");
        launchMotor_2 = hardwareMap.get(DcMotorEx.class, "LaunchMotor_2");
        battery = hardwareMap.voltageSensor.iterator().next();
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");


        feedServo.setPower(0.0);
        isFeedServoDown = false;

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor_2.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // intake runs open-loop (no encoder feedback)
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Drive (verify your constructor signature)
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Vision
        tagService = new AprilTagService();
        // tagService.start(hardwareMap);

        // Seed dashboard values with current motor PIDF so you see "real" starting numbers
        PIDFCoefficients pidf_cur = launchMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        DashTuning.P = pidf_cur.p;
        DashTuning.I = pidf_cur.i;
        DashTuning.D = pidf_cur.d;
        DashTuning.F = pidf_cur.f;

        // LOG: create CSV logger
        if (LOG_ENABLED) {
            logger = TinyCsvLoggerFlex.create(
                    hardwareMap,
                    "teleop_main",
                    TinyCsvLoggerFlex.doubleCol("launch_cmd", () -> (autoShooter ? shooterSetpointTPS : launchPowerVel)),
                    TinyCsvLoggerFlex.motorEx("launch", launchMotor),
                    TinyCsvLoggerFlex.doubleCol("intake_cmd", () -> intakePower),
                    TinyCsvLoggerFlex.motorEx("intake", intakeMotor),
                    TinyCsvLoggerFlex.pose2d("pose", () -> drive.localizer.getPose())
            );
        }

        telemetry.addLine("Dashboard PIDF tuning enabled.");
        telemetry.addLine("Open FTC Dashboard -> Config -> TeleOpMainPIDFTunner -> DashTuning");
        telemetry.update();

        waitForStart();

        // Safe startup
        intakeMotor.setPower(0.0);
        launchMotor.setPower(0.0);
        launchMotor_2.setPower(0.0);
        feedServo.setPower(0.0);

        double vel_error = 0.0;

        while (opModeIsActive()) {

            // ---------------- Apply PIDF from Dashboard ----------------
            PIDFCoefficients cur = new PIDFCoefficients(
                    DashTuning.P,
                    DashTuning.I,
                    DashTuning.D,
                    DashTuning.F
            );

            if (DashTuning.applyPidfContinuously) {
                launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, cur);
                launchMotor_2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, cur);
            } else {
                if (lastAppliedPidf == null ||
                        lastAppliedPidf.p != cur.p ||
                        lastAppliedPidf.i != cur.i ||
                        lastAppliedPidf.d != cur.d ||
                        lastAppliedPidf.f != cur.f) {
                    launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, cur);
                    launchMotor_2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, cur);
                    lastAppliedPidf = cur;
                }
            }

            // # # # Gamepad 1 (Driver) # # #
            // -------------------------------- Base Drive -----------------------------------------
            if (gamepad1.a) speedFactor = 1.0;
            if (gamepad1.b) speedFactor = 0.4;
            if (gamepad1.x) speedFactor = 0.7;

            double axial = -gamepad1.right_stick_y * speedFactor;
            double lateral = -gamepad1.left_stick_x * speedFactor;
            double heading = -gamepad1.right_stick_x * speedFactor;

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(axial, lateral), heading));
            drive.updatePoseEstimate();

            Pose2d pose = drive.localizer.getPose();
            telemetry.addData("Speed Factor", "%.2f", speedFactor);

            // \--- Vision toggle (edge-based, no sleep) ---
            boolean g1DownEdge = gamepad1.dpad_down && !prevG1DpadDown;
            if (g1DownEdge) {
                if (visionEnabled) {
                    tagService.stop();
                    visionEnabled = false;
                } else {
                    tagService.start(hardwareMap);
                    visionEnabled = true;
                }
            }
            prevG1DpadDown = gamepad1.dpad_down;

            // # # # Gamepad 2 (Controls) # # #
            // --------------------------- MODE TOGGLES -------------------------
            boolean upEdge = gamepad2.dpad_up && !prevDpadUp;
            boolean downEdge = gamepad2.dpad_down && !prevDpadDown;
            if (upEdge) {
                autoShooter = true;
                autoSpinArmed = false;
                launchMotor.setPower(0.0);
                launchMotor_2.setPower(0.0);
            }
            if (downEdge) {
                autoShooter = false;
                autoSpinArmed = false;
                launchMotor.setPower(0.0);
                launchMotor_2.setPower(0.0);
            }
            prevDpadUp = gamepad2.dpad_up;
            prevDpadDown = gamepad2.dpad_down;

            // Dpad-right → arm auto spin
            boolean rightEdge = gamepad2.dpad_right && !prevDpadRight;
            if (rightEdge && autoShooter) {
                autoSpinArmed = !autoSpinArmed;
            }
            prevDpadRight = gamepad2.dpad_right;

            // --------------------------- MANUAL MODE --------------------------
            if (!autoShooter) {
                // Manual command is now driven by Dashboard
                double cmd = manualTargetTPS;
                cmd = Math.max(0.0, Math.min(cmd, DashTuning.manualMaxTPS));

                launchPowerVel = cmd;
                launchMotor.setVelocity(launchPowerVel);
                launchMotor_2.setVelocity(launchPowerVel);
                vel_error = launchPowerVel - launchMotor.getVelocity();

                // Optional quick kill on X
                if (gamepad2.x) {
                    launchPowerVel = 0.0;
                    launchMotor.setPower(0.0);
                    launchMotor_2.setPower(0.0);
                    vel_error = 0.0;
                }
            }

            // --------------------------- AUTO MODE ----------------------------
           /* if (autoShooter) {
                if (autoSpinArmed) {
                    Double dInches = getVisionDistanceInches();
                    if (dInches != null && dInches >= ShooterConfig.MIN_RANGE_IN) {
                        double tps = Calculations.computeTPSFromRangeInches(
                                ShooterConfig.G, dInches,
                                ShooterConfig.LAUNCH_DEG,
                                ShooterConfig.SHOOTER_H_M,
                                ShooterConfig.TARGET_H_M,
                                ShooterConfig.WHEEL_RADIUS_M,
                                ShooterConfig.EFFICIENCY,
                                ShooterConfig.TICKS_PER_REV
                        );
                        if (!Double.isNaN(tps) && Double.isFinite(tps)) {
                            if (ShooterConfig.TEST_TPS > 0) {
                                tps = ShooterConfig.TEST_TPS;
                            }
                            tps = Math.min(tps, ShooterConfig.TPS_MAX);
                            shooterSetpointTPS = tps;
                            launchMotor.setVelocity(tps);
                            launchMotor_2.setVelocity(tps);

                        } else {
                            shooterSetpointTPS = 0.0;
                            launchMotor.setPower(0.0);
                            launchMotor_2.setPower(0.0);
                        }
                    } else {
                        shooterSetpointTPS = 0.0;
                        launchMotor.setPower(0.0);

                        launchMotor_2.setPower(0.0);
                    }
                } else {
                    shooterSetpointTPS = 0.0;
                    launchMotor.setPower(0.0);
                    launchMotor_2.setPower(0.0);
                }
            }*/

            // --------------------------- FEED LOGIC ---------------------------
            boolean spunUpOk = false;
            if (autoShooter && shooterSetpointTPS > 0.0) {
                double vel = launchMotor.getVelocity();
                double vel2 = launchMotor.getVelocity();
                spunUpOk = Math.abs(vel - shooterSetpointTPS) <= ShooterConfig.TPS_TOL &&  Math.abs(vel2 - shooterSetpointTPS) <= ShooterConfig.TPS_TOL;
            } else if (!autoShooter) {
                spunUpOk = (launchMotor.getPower() > 0.0);
                // FIXED: manual uses velocity control, so compare velocity error not getPower()
                spunUpOk = (launchPowerVel > 0.0) &&
                        (Math.abs(launchMotor.getVelocity() - launchPowerVel) <= DashTuning.manualTolTPS) &&
                        (Math.abs(launchMotor_2.getVelocity() - launchPowerVel) <= DashTuning.manualTolTPS);
            }

            if (gamepad2.y) {
                if (!feedPulseActive && spunUpOk) {
                    feedServo.setPower(-0.75);//0.16
                    feedPulseActive = true;
                } else if(feedPulseActive){
                    feedServo.setPower(0.0);
                    feedPulseActive = false;
                }

                // --------------------------- INTAKE -------------------------------
                boolean rbEdge = gamepad2.right_bumper && !prevRB;
                if (rbEdge) intakePower = -0.5;
                prevRB = gamepad2.right_bumper;

                if (gamepad2.right_trigger > 0) intakePower = 0.73;
                if (gamepad2.left_trigger > 0) intakePower = 0.0;

                intakeMotor.setPower(intakePower);

                // --------------------------- LED STATES ---------------------------
                RevBlinkinLedDriver.BlinkinPattern pat = RevBlinkinLedDriver.BlinkinPattern.BLACK;
                AprilTagService.Reading r = tagService.getLatest();
                boolean hasTag = (r != null && r.hasTag);
                boolean correctTag = hasTag && (r.id == GOAL_TAG_ID);

                if (!visionEnabled) {
                    pat = RevBlinkinLedDriver.BlinkinPattern.BLACK;
                } else if (!correctTag) {
                    pat = RevBlinkinLedDriver.BlinkinPattern.RED;
                } else {
                    boolean atSpeed = spunUpOk && autoSpinArmed && autoShooter;
                    pat = atSpeed ? RevBlinkinLedDriver.BlinkinPattern.GREEN
                            : RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                }
                if (System.nanoTime() < yTooSoonFlashUntilNs) {
                    pat = RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD;
                }

                // --------------------------- LOGGING ------------------------------
                if (LOG_ENABLED && logger != null) {
                    logger.record("run");
                }

            }

            // ------------- Telemetry data -------------------------------------------------
            double tpsMeas = launchMotor.getVelocity();
            double rpmMeas = (tpsMeas * 60.0) / ShooterConfig.TICKS_PER_REV;
            AprilTagService.Reading r = tagService.getLatest();
            // Double visInches = getVisionDistanceInches();
            int tagId = (r != null && r.hasTag) ? r.id : -1;

            telemetry.addLine("---- Shooter ----");
            telemetry.addData("Mode", autoShooter ? "AUTO" : "MANUAL");
            telemetry.addData("Armed", autoSpinArmed);
            telemetry.addData("Setpoint TPS", "%.0f", shooterSetpointTPS);

            telemetry.addData("Dash PIDF", "P=%.4f I=%.4f D=%.4f F=%.4f",
                    DashTuning.P, DashTuning.I, DashTuning.D, DashTuning.F);

            if (autoShooter) {
                telemetry.addData("Setpoint TPS", "%.0f", shooterSetpointTPS);
            } else {
                telemetry.addData("Manual TPS Cmd (Dash)", "%.0f", launchPowerVel);
            }

            telemetry.addData("Actual TPS", "%.0f", tpsMeas);
            telemetry.addData("Measured RPM", "%.0f", rpmMeas);
            telemetry.addData("Velocity Error", "%.0f", vel_error);
            if (!autoShooter) telemetry.addData("Manual Power", "%.2f", CompPower);
            telemetry.addData("Ready?", spunUpOk);
            telemetry.addData("Manual TPS", manualTargetTPS);
            telemetry.addData("LaunchMotor", launchMotor.getVelocity());
            telemetry.addData("LaunchMotor_2", launchMotor_2.getVelocity());

            telemetry.addLine("---- Vision ----");
            telemetry.addData("Vision Enabled", visionEnabled);
            telemetry.addData("Tag ID", tagId);
            telemetry.addData("Goal Tag ID", GOAL_TAG_ID);
     //       telemetry.addData("Correct Tag", correctTag);
            //  telemetry.addData("Range (in)", (visInches == null) ? "N/A" : String.format("%.1f", visInches));
       //     telemetry.addData("LED", pat.name());

            telemetry.update();

            // cleanup
            try {
                launchMotor.setPower(0.0);
                launchMotor_2.setPower(0.0);
                intakeMotor.setPower(0.0);
            } finally {
                tagService.stop();
                if (LOG_ENABLED && logger != null) logger.close();
            }
        }
    }
}