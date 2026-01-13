// TeleOpMainBlue.java
// Full file with Option A implemented AND feed gating always includes "wrong angle".
// Removed all REQUIRE_ALIGNED_TO_FEED references (does not exist anymore).

package org.firstinspires.ftc.teamcode.MainCode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MainCode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.MainCode.util.Calculations;
import org.firstinspires.ftc.teamcode.MainCode.util.TinyCsvLoggerFlex;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@TeleOp(name = "TeleOpRed: Main", group = "TeleOp")
public class TeleOpMainRed extends LinearOpMode {

    // --- Hardware ---
    private Servo feedServo;
    private MecanumDrive drive;
    private DcMotorEx intakeMotor;
    private DcMotorEx launchMotor;
    private RevBlinkinLedDriver blinkin;
    private VoltageSensor battery;

    // --- Vision ---
    private boolean visionEnabled = true;

    // Auto shooter (closed-loop velocity)
    private boolean autoShooter = true;
    private double shooterSetpointTPS = 0.0;
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
    private double finalTps_dbg = 0.0;       // actual TPS commanded to motor
    private boolean noShotZone_dbg = false;

    // --- Logging ---
    private static final boolean LOG_ENABLED = true;
    private TinyCsvLoggerFlex logger;

    private static final int GOAL_TAG_ID = 24; // 20 = blue goal, 24 = red goal

    // require driver to arm auto-spin before controlling flywheel
    private boolean autoSpinArmed = true;
    private boolean prevDpadRight = false;

    // flash window when Y pressed too soon OR no-shot zone
    private long yTooSoonFlashUntilNs = 0L;
    private static final long FLASH_YELLOW_NS = 500_000_000L;

    // --- Drive/settings ---
    private double speedFactor = 1.2;
    @SuppressWarnings("unused")
    BNO055IMU imu;

    // --- Intake/servo state ---
    private double intakePower = 0.0;

    // --- Button edge detection ---
    private boolean prevRB = false;

    private boolean feedPulseActive = false;
    private boolean intakeMotorPulseActive = false;
    private long feedPulseStartNs = 0;
    private double intakePulseStartNs = 0;
    private static final long FEED_DWELL_NS = 150_000_000L;
    private static final double INTAKE_DWELL_NS = 1000000000;

    // edge state for GP2 dpad-Left (vision toggle)
    private boolean prevG2DpadLeft = false;

    private static final double M_TO_IN = 39.37007874015748;

    // ---------------- Auto-align state (added) ----------------
    private double prevAlignErr = 0.0;
    private long prevAlignNs = 0L;

    // Debug values for telemetry (added)
    private double txMin_dbg = 0.0;
    private double txMax_dbg = 0.0;
    private double txTarget_dbg = 0.0;
    private double alignErr_dbg = 0.0;
    private boolean alignActive_dbg = false;

    @Override
    public void runOpMode() {

        // Map hardware
        feedServo   = hardwareMap.get(Servo.class,     "feedServo");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        launchMotor = hardwareMap.get(DcMotorEx.class, "LaunchMotor");
        battery     = hardwareMap.voltageSensor.iterator().next();

        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        feedServo.setPosition(0.02);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Drive
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // LOG: create CSV logger
        if (LOG_ENABLED) {
            logger = TinyCsvLoggerFlex.create(
                    hardwareMap,
                    "teleop_main",
                    TinyCsvLoggerFlex.doubleCol("dist_raw_in", () -> distIn_raw_dbg),
                    TinyCsvLoggerFlex.doubleCol("dist_filt_in", () -> distIn_filt_dbg),
                    TinyCsvLoggerFlex.doubleCol("tps_physics", () -> physicsTps_dbg),
                    TinyCsvLoggerFlex.doubleCol("tps_table", () -> tableTps_dbg),
                    TinyCsvLoggerFlex.doubleCol("tps_base_cmd", () -> commandedBase_dbg),
                    TinyCsvLoggerFlex.doubleCol("tps_final_cmd", () -> finalTps_dbg),
                    TinyCsvLoggerFlex.doubleCol("launch_cmd", () -> shooterSetpointTPS),
                    TinyCsvLoggerFlex.motorEx("launch", launchMotor),
                    TinyCsvLoggerFlex.doubleCol("intake_cmd", () -> intakePower),
                    TinyCsvLoggerFlex.motorEx("intake", intakeMotor),
                    TinyCsvLoggerFlex.servoPos("feed_pos", feedServo),
                    TinyCsvLoggerFlex.pose2d("pose", () -> drive.localizer.getPose())
            );
        }

        waitForStart();

        intakeMotor.setPower(0.0);
        launchMotor.setPower(0.0);
        PIDFCoefficients pidf_cur = new PIDFCoefficients(500, 3, 0, 4);
        launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf_cur);

        while (opModeIsActive()) {

            // ---------------- Base Drive ----------------
            if (gamepad1.a) speedFactor = 1.2;
            if (gamepad1.b) speedFactor = 0.4;
            if (gamepad1.x) speedFactor = 0.7;

            double axial   = -gamepad1.right_stick_y * speedFactor;
            double lateral = -gamepad1.left_stick_x  * speedFactor;
            double headingManual = -gamepad1.right_stick_x * speedFactor;

            // ---------------- Vision toggle ----------------
            boolean g2LeftEdge = gamepad2.dpad_left && !prevG2DpadLeft;
            if (g2LeftEdge) visionEnabled = !visionEnabled;
            prevG2DpadLeft = gamepad2.dpad_left;

            // ---------------- Vision Read ----------------
            LLResult ll = null;
            if (visionEnabled) {
                LLResult tmp = limelight.getLatestResult();
                if (tmp != null && tmp.isValid() && tmp.getStaleness() < 100) {
                    ll = tmp;
                }
            }

            // Update Tx/Ty ASAP so anything below doesn't use stale values
            if (ll != null) {
                Tx = ll.getTx();
                Ty = ll.getTy();
            }

            Double visInches = getVisionDistanceInches(ll);

            // distance smoothing (optional)
            distIn_raw_dbg = (visInches != null) ? visInches : 0.0;
            if (visInches != null) {
                if (distIn_filt_dbg <= 0.0) distIn_filt_dbg = visInches; // init
                double a = ShooterConfig.DIST_SMOOTH_ALPHA;
                distIn_filt_dbg = (1.0 - a) * distIn_filt_dbg + a * visInches;
            }

            // NO-SHOT ZONE (measured "too close" distance where shot is impossible)
            Double dInForLogic = (visInches != null) ? distIn_filt_dbg : null;
            boolean noShotZone =
                    (dInForLogic != null)
                            && ShooterConfig.NO_SHOT_UNDER_IN > 0.0
                            && dInForLogic < ShooterConfig.NO_SHOT_UNDER_IN;
            noShotZone_dbg = noShotZone;

            // tag correctness / LED info
            boolean correctTag = false;
            int tagId = -1;
            if (ll != null) {
                List<LLResultTypes.FiducialResult> fiducials = ll.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    int firstId = -1;
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (f == null) continue;
                        if (firstId == -1) firstId = f.getFiducialId();

                        if (f.getFiducialId() == GOAL_TAG_ID) {
                            correctTag = true;
                            tagId = GOAL_TAG_ID;
                            break;
                        }
                    }
                    if (!correctTag) tagId = firstId;
                }
            }

            // ---------------- Auto Align ----------------
            // Hold to auto-align yaw into distance-based Tx window.
            // Button choice: gamepad1.left_bumper (change if you want)
            alignActive_dbg = false;
            txMin_dbg = 0.0;
            txMax_dbg = 0.0;
            txTarget_dbg = 0.0;
            alignErr_dbg = 0.0;

            double headingCmd = headingManual;

            boolean alignBtn = gamepad1.left_bumper;
            boolean fresh = (ll != null && ll.isValid() && ll.getStaleness() < ShooterConfig.ALIGN_MAX_STALE_MS);

            if (ShooterConfig.AUTO_ALIGN_ENABLED
                    && alignBtn
                    && visionEnabled
                    && fresh
                    && hasGoalTag_dbg) {

                double tx = Tx;

                // Use same distance you already use for shooter logic (filtered)
                double dIn = (visInches != null) ? distIn_filt_dbg : Double.NaN;
                double[] win = ShooterConfig.lookupTxWindowFromDistanceIn(dIn);
                double txMin = win[0];
                double txMax = win[1];

                txMin_dbg = txMin;
                txMax_dbg = txMax;

                // Choose nearest boundary only if outside window; if inside, don't rotate.
               /* if (tx < txMin) {
                    txTarget_dbg = txMin;
                } else if (tx > txMax) {
                    txTarget_dbg = txMax;*/
                if(tx < txMin || tx > txMax){
                    txTarget_dbg = (txMin + txMax)/2;
                } else {
                    txTarget_dbg = tx; // already within range
                }

                double err =  txTarget_dbg- tx;   // want err -> 0
                alignErr_dbg = err;

                if (Math.abs(err) <= ShooterConfig.ALIGN_ERR_DEADBAND_DEG) {
                    headingCmd = 0.0;
                } else {
                    headingCmd = computeAlignTurnFromErr(err);
                }

                alignActive_dbg = true;
            } else {
                // reset derivative timing when not aligning
                prevAlignNs = 0L;
                prevAlignErr = 0.0;
            }

            // Apply drive AFTER align logic
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(axial, lateral), headingCmd));
            drive.updatePoseEstimate();

            // ---------------- Arm toggle ----------------
            boolean rightEdge = gamepad2.dpad_right && !prevDpadRight;
            if (rightEdge && autoShooter) autoSpinArmed = !autoSpinArmed;
            prevDpadRight = gamepad2.dpad_right;

            // ---------------- AUTO SHOOTER ----------------
            physicsTps_dbg = 0.0;
            tableTps_dbg = 0.0;
            commandedBase_dbg = 0.0;
            finalTps_dbg = 0.0;

            if (autoShooter && autoSpinArmed) {

                Double dIn = (visInches != null) ? distIn_filt_dbg : null;

                // Require: valid distance, above ignore floor, AND not in no-shot zone
                if (dIn != null && dIn >= ShooterConfig.MIN_RANGE_IN && !noShotZone) {

                    // 1) physics TPS (base)
                    double physicsTps = Calculations.computeTPSFromRangeInches(
                            ShooterConfig.G,
                            dIn,
                            ShooterConfig.LAUNCH_DEG,
                            ShooterConfig.SHOOTER_H_M,
                            ShooterConfig.TARGET_H_M,
                            ShooterConfig.WHEEL_RADIUS_M,
                            ShooterConfig.EFFICIENCY,
                            ShooterConfig.TICKS_PER_REV
                    );
                    if (!Double.isFinite(physicsTps)) physicsTps = 0.0;
                    physicsTps_dbg = physicsTps;

                    // 2) table TPS (base)
                    double tableTps = ShooterConfig.lookupTpsFromDistanceIn(dIn);
                    tableTps_dbg = tableTps;

                    // 3) choose base TPS (ONE SWITCH)
                    boolean useTable = ShooterConfig.USE_TABLE;
                    double base = useTable ? tableTps_dbg : physicsTps_dbg;
                    commandedBase_dbg = base;

                    // 4) final TPS:
                    double desired = useTable
                            ? ShooterConfig.clampTps(base)
                            : ShooterConfig.applyTuningAndClamp(base);

                    finalTps_dbg = desired;

                    if (desired <= 0.0) {
                        shooterSetpointTPS = 0.0;
                        launchMotor.setPower(0.0);
                    } else {
                        shooterSetpointTPS = desired;
                        launchMotor.setVelocity(shooterSetpointTPS);
                    }

                } else {
                    shooterSetpointTPS = 0.0;
                    launchMotor.setPower(0.0);
                }

            } else {
                shooterSetpointTPS = 0.0;
                launchMotor.setPower(0.0);
            }

            // ---------------- FEED LOGIC ----------------
            boolean spunUpOk = false;
            if (autoShooter && shooterSetpointTPS > 0.0) {
                double vel = launchMotor.getVelocity();
                spunUpOk = Math.abs(vel - shooterSetpointTPS) <= ShooterConfig.TPS_TOL;
            }

            // Angle gate: only allow feed if tag is found AND Tx is within the distance-based window.
            boolean angleOk = false;
            if (visionEnabled && hasGoalTag_dbg && ll != null && ll.isValid() && ll.getStaleness() < 100) {
                double[] win = ShooterConfig.lookupTxWindowFromDistanceIn(distIn_filt_dbg);
                double txMin = win[0];
                double txMax = win[1];
                angleOk = (Tx >= txMin && Tx <= txMax);
            }

            // Block feeding if not spun up OR in no-shot zone OR not within angle window
            boolean feedAllowed = spunUpOk && !noShotZone;

            if (gamepad2.y) {
                if (!feedPulseActive && feedAllowed) {
                    feedServo.setPosition(0.12);
                    feedPulseActive = true;
                    feedPulseStartNs = System.nanoTime();
                    if(!intakeMotorPulseActive) {
                        intakeMotorPulseActive = true;
                        intakePulseStartNs = System.nanoTime();
                        intakeMotor.setPower(1.0);
                    }

                } else if (!feedAllowed) {
                    yTooSoonFlashUntilNs = System.nanoTime() + FLASH_YELLOW_NS;
                }
            }

            if (feedPulseActive && System.nanoTime() - feedPulseStartNs >= FEED_DWELL_NS) {
                feedServo.setPosition(0.02);
                feedPulseActive = false;
            }

            if(intakeMotorPulseActive && System.nanoTime() - intakePulseStartNs >= INTAKE_DWELL_NS){
                intakeMotor.setPower(0.0);
                intakeMotorPulseActive = false;
            }

            // ---------------- INTAKE ----------------
            boolean rbEdge = gamepad2.right_bumper && !prevRB;
            if (rbEdge) intakePower = -0.5;
            prevRB = gamepad2.right_bumper;

            if (gamepad2.right_trigger > 0) intakePower = 0.73;
            if (gamepad2.left_trigger > 0) intakePower = 0.0;
            intakeMotor.setPower(intakePower);

            // ---------------- LEDs ----------------
            RevBlinkinLedDriver.BlinkinPattern pat = RevBlinkinLedDriver.BlinkinPattern.BLACK;

            if (!visionEnabled) {
                pat = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            } else if (autoShooter) {
                if (!correctTag) {
                    pat = RevBlinkinLedDriver.BlinkinPattern.RED;
                } else if (noShotZone) {
                    // Too close to make the shot: force yellow even if at speed
                    pat = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                } else {
                    boolean atSpeed = spunUpOk && autoSpinArmed;
                    pat = atSpeed ? RevBlinkinLedDriver.BlinkinPattern.GREEN
                            : RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                }
            }

            // If Y pressed when not allowed (not at speed OR no-shot zone), flash gold
            if (System.nanoTime() < yTooSoonFlashUntilNs) {
                pat = RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD;
            }
            blinkin.setPattern(pat);

            // ---------------- LOGGING ----------------
            if (LOG_ENABLED && logger != null) {
                logger.record("run");
            }

            // ---------------- TELEMETRY ----------------
            telemetry.addLine("---- Vision Distance (cameraPoseTargetSpace) ----");
            telemetry.addData("Vision Enabled", visionEnabled);
            telemetry.addData("Goal Tag Found", hasGoalTag_dbg);
            telemetry.addData("Seen Tag ID", tagId);
            telemetry.addData("GOAL_TAG_ID", GOAL_TAG_ID);
            telemetry.addData("rangeRawIn", "%.2f", distIn_raw_dbg);
            telemetry.addData("rangeFiltIn", "%.2f", distIn_filt_dbg);
            telemetry.addData("Tx",  Tx);
            telemetry.addData("Ty", Ty);

            telemetry.addLine("---- Align Window (Tx) ----");
            telemetry.addData("AlignActive", alignActive_dbg);
            telemetry.addData("txMin", "%.2f", txMin_dbg);
            telemetry.addData("txMax", "%.2f", txMax_dbg);
            telemetry.addData("txTarget", "%.2f", txTarget_dbg);
            telemetry.addData("alignErr", "%.2f", alignErr_dbg);

            telemetry.addLine("---- No-Shot Zone ----");
            telemetry.addData("NO_SHOT_UNDER_IN", "%.2f", ShooterConfig.NO_SHOT_UNDER_IN);
            telemetry.addData("noShotZone", noShotZone_dbg);

            telemetry.addLine("---- Shooter Calc ----");
            telemetry.addData("USE_TABLE (match)", ShooterConfig.USE_TABLE);
            telemetry.addData("physicsTPS", "%.0f", physicsTps_dbg);
            telemetry.addData("tableTPS", "%.0f", tableTps_dbg);
            telemetry.addData("baseChosen", "%.0f", commandedBase_dbg);

            telemetry.addData("scale (physics only)", "%.3f", ShooterConfig.TPS_SCALE);
            telemetry.addData("offset (physics only)", "%.0f", ShooterConfig.TPS_OFFSET);
            telemetry.addData("finalTPS", "%.0f", finalTps_dbg);

            telemetry.addLine("---- Shooter State ----");
            telemetry.addData("Armed", autoSpinArmed);
            telemetry.addData("Setpoint TPS", "%.0f", shooterSetpointTPS);
            telemetry.addData("Actual TPS", "%.0f", launchMotor.getVelocity());
            telemetry.addData("Err", "%.0f", (launchMotor.getVelocity() - shooterSetpointTPS));
            telemetry.addData("Ready", spunUpOk);
            telemetry.addData("FeedAllowed", feedAllowed);

            telemetry.addLine("TIP: Set NO_SHOT_UNDER_IN to your measured 'too close' distance.");
            telemetry.update();
        }

        try {
            launchMotor.setPower(0.0);
            intakeMotor.setPower(0.0);
        } finally {
            if (LOG_ENABLED && logger != null) logger.close();
        }
    }

    // ---------------- Align helper (added) ----------------
    private double computeAlignTurnFromErr(double errDeg) {
        long now = System.nanoTime();
        double dt = (prevAlignNs == 0L) ? 0.0 : (now - prevAlignNs) / 1e9;
        prevAlignNs = now;

        double derr = 0.0;
        if (dt > 1e-4) derr = (errDeg - prevAlignErr) / dt;
        prevAlignErr = errDeg;

        double u = ShooterConfig.ALIGN_KP * errDeg + ShooterConfig.ALIGN_KD * derr;

        if (u > ShooterConfig.ALIGN_MAX_TURN) u = ShooterConfig.ALIGN_MAX_TURN;
        if (u < -ShooterConfig.ALIGN_MAX_TURN) u = -ShooterConfig.ALIGN_MAX_TURN;

        if (Math.abs(u) > 0.0 && Math.abs(u) < ShooterConfig.ALIGN_MIN_TURN) {
            u = Math.copySign(ShooterConfig.ALIGN_MIN_TURN, u);
        }

        return u;
    }

    /**
     * Returns distance in inches using Limelight cameraPoseTargetSpace:
     * range = sqrt(x^2 + z^2)
     */
    private Double getVisionDistanceInches(LLResult result) {
        hasGoalTag_dbg = false;

        xM_dbg = yM_dbg = zM_dbg = 0;
        xIn_dbg = yIn_dbg = zIn_dbg = 0;
        rangeIn_dbg = 0;

        if (result == null || !result.isValid() || result.getStaleness() >= 100) return null;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return null;

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial == null) continue;
            if (fiducial.getFiducialId() != GOAL_TAG_ID) continue;

            Pose3D targetPose = fiducial.getCameraPoseTargetSpace();
            if (targetPose == null) continue;

            hasGoalTag_dbg = true;

            xM_dbg = targetPose.getPosition().x;
            yM_dbg = targetPose.getPosition().y;
            zM_dbg = targetPose.getPosition().z;

            xIn_dbg = xM_dbg * M_TO_IN;
            yIn_dbg = yM_dbg * M_TO_IN;
            zIn_dbg = zM_dbg * M_TO_IN;

            double rangeM = Math.sqrt((xM_dbg * xM_dbg) + (zM_dbg * zM_dbg));
            rangeIn_dbg = rangeM * M_TO_IN;

            return rangeIn_dbg;
        }

        return null;
    }
}