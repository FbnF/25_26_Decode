package org.firstinspires.ftc.teamcode.MainCode.util;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MainCode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

/**
 * Reusable Road Runner Actions:
 *  - setMotorPower(...) : one-shot, non-blocking motor power setter
 *  - setMotorVel(...)   : one-shot, non-blocking motor velocity setter (TPS)
 *  - setServoPosition(...) : one-shot, non-blocking servo setter
 *
 * Legacy Actions:
 *  - TimedMotorPowerAction
 *  - ServoPulseAction
 *  - ServoScheduleAction
 *  - ShooterAndFeederAction       : shooter POWER + scheduled feed windows (legacy)
 *  - ShooterAndFeederActionVel    : shooter VELOCITY (TPS) + scheduled feed windows (legacy)
 *
 * Vision Action:
 *  - ShooterAndFeederVisionAction : Limelight distance->TPS table + gated feeding
 *
 * IMPORTANT (your RR flavor): Action.run() returns TRUE to keep running, FALSE when finished.
 */
public final class AutoMotorControl {

    private AutoMotorControl() {}

    /** One-shot action that sets a motor's power and immediately completes (non-blocking). */
    public static Action setMotorPower(DcMotor m, double power) {
        return (TelemetryPacket pkt) -> {
            if (m != null) {
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m.setPower(power);
            }
            return false;
        };
    }

    /** One-shot action that sets a motor's velocity (TPS) and immediately completes (non-blocking). */
    public static Action setMotorVel(DcMotorEx m, double tps) {
        return (TelemetryPacket pkt) -> {
            if (m != null) {
                m.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                m.setVelocity(tps);
            }
            return false;
        };
    }

    /** One-shot action that sets a servo's position and immediately completes (non-blocking). */
    public static Action setServoPosition(Servo s, double pos) {
        return (TelemetryPacket pkt) -> {
            if (s != null) s.setPosition(pos);
            return false;
        };
    }

    // -------------------------------------------------------------------------
    // Legacy utility actions
    // -------------------------------------------------------------------------

    public static class TimedMotorPowerAction implements Action {
        private final DcMotorEx motor;
        private final double power;
        private final double durationS;
        private final boolean stopAtEnd;

        private boolean initialized = false;
        private long t0;

        public TimedMotorPowerAction(DcMotorEx motor, double power, double durationS, boolean stopAtEnd) {
            this.motor = motor;
            this.power = power;
            this.durationS = durationS;
            this.stopAtEnd = stopAtEnd;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                t0 = System.nanoTime();
                if (motor != null) {
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motor.setPower(power);
                }
                initialized = true;
            }

            double t = (System.nanoTime() - t0) / 1e9;
            packet.put("timedMotor_t_s", String.format("%.2f", t));

            if (t < durationS) return true;

            if (motor != null && stopAtEnd) motor.setPower(0.0);
            return false;
        }
    }

    public static class ServoPulseAction implements Action {
        private final Servo servo;
        private final double loadPos;
        private final double feedPos;

        private final double startDelayS;
        private final double holdS;
        private final double gapS;
        private final int repeats;
        private final double endPaddingS;

        private boolean initialized = false;
        private long t0;

        public ServoPulseAction(
                Servo servo,
                double loadPos,
                double feedPos,
                double startDelayS,
                double holdS,
                double gapS,
                int repeats,
                double endPaddingS
        ) {
            this.servo = servo;
            this.loadPos = loadPos;
            this.feedPos = feedPos;
            this.startDelayS = Math.max(0, startDelayS);
            this.holdS = Math.max(0, holdS);
            this.gapS = Math.max(0, gapS);
            this.repeats = Math.max(0, repeats);
            this.endPaddingS = Math.max(0, endPaddingS);
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                t0 = System.nanoTime();
                if (servo != null) servo.setPosition(loadPos);
                initialized = true;
            }

            double t = (System.nanoTime() - t0) / 1e9;
            boolean feeding = false;

            if (t >= startDelayS && repeats > 0) {
                double tSinceStart = t - startDelayS;
                double period = Math.max(holdS + gapS, holdS);
                int idx = (int) Math.floor(tSinceStart / period);
                if (idx < repeats) {
                    double tIntoPulse = tSinceStart - idx * period;
                    feeding = (tIntoPulse >= 0 && tIntoPulse < holdS);
                }
            }

            if (servo != null) servo.setPosition(feeding ? feedPos : loadPos);

            packet.put("servoPulse_t_s", String.format("%.2f", t));
            packet.put("servoPulse_feeding", feeding);

            double totalActive = startDelayS + (repeats > 0 ? (repeats * (holdS + gapS)) - gapS : 0);
            double lastEnd = totalActive + endPaddingS;

            if (t < lastEnd) return true;

            if (servo != null) servo.setPosition(loadPos);
            return false;
        }
    }

    public static class ServoScheduleAction implements Action {
        private final Servo servo;
        private final double loadPos;
        private final double feedPos;
        private final double[] feedStartS;
        private final double feedHoldS;
        private final double endPaddingS;

        private boolean initialized = false;
        private long t0;

        public ServoScheduleAction(
                Servo servo,
                double loadPos,
                double feedPos,
                double[] feedStartS,
                double feedHoldS,
                double endPaddingS
        ) {
            this.servo = servo;
            this.loadPos = loadPos;
            this.feedPos = feedPos;
            this.feedStartS = (feedStartS != null) ? feedStartS : new double[0];
            this.feedHoldS = Math.max(0, feedHoldS);
            this.endPaddingS = Math.max(0, endPaddingS);
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                t0 = System.nanoTime();
                if (servo != null) servo.setPosition(loadPos);
                initialized = true;
            }

            double t = (System.nanoTime() - t0) / 1e9;

            boolean feeding = false;
            for (double s : feedStartS) {
                if (t >= s && t < s + feedHoldS) {
                    feeding = true;
                    break;
                }
            }
            if (servo != null) servo.setPosition(feeding ? feedPos : loadPos);

            packet.put("servoSched_t_s", String.format("%.2f", t));
            packet.put("servoSched_feeding", feeding);

            double lastEnd = (feedStartS.length > 0 ? feedStartS[feedStartS.length - 1] : 0.0)
                    + feedHoldS + endPaddingS;

            if (t < lastEnd) return true;

            if (servo != null) servo.setPosition(loadPos);
            return false;
        }
    }

    // -------------------------------------------------------------------------
    // Legacy shooter actions (kept for existing autos)
    // -------------------------------------------------------------------------

    /** Legacy: shooter POWER + scheduled feed windows. */
    public static class ShooterAndFeederAction implements Action {
        private final DcMotorEx shooter;
        private final Servo feeder;
        private final double shooterPower;
        private final double[] feedStartS;
        private final double feedHoldS;
        private final double endPaddingS;
        private final double loadPos;
        private final double feedPos;

        private boolean initialized = false;
        private long t0;

        public ShooterAndFeederAction(
                DcMotorEx shooter,
                Servo feeder,
                double shooterPower,
                double[] feedStartS,
                double feedHoldS,
                double endPaddingS,
                double loadPos,
                double feedPos
        ) {
            this.shooter = shooter;
            this.feeder = feeder;
            this.shooterPower = shooterPower;
            this.feedStartS = (feedStartS != null) ? feedStartS : new double[0];
            this.feedHoldS = Math.max(0.0, feedHoldS);
            this.endPaddingS = Math.max(0.0, endPaddingS);
            this.loadPos = loadPos;
            this.feedPos = feedPos;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                t0 = System.nanoTime();
                if (shooter != null) {
                    shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    shooter.setPower(shooterPower);
                }
                if (feeder != null) feeder.setPosition(loadPos);
                initialized = true;
            }

            double t = (System.nanoTime() - t0) / 1e9;

            boolean feeding = false;
            for (double s : feedStartS) {
                if (t >= s && t < s + feedHoldS) { feeding = true; break; }
            }
            if (feeder != null) feeder.setPosition(feeding ? feedPos : loadPos);

            packet.put("t_s", String.format("%.2f", t));
            packet.put("feeding", feeding);

            double lastEnd = (feedStartS.length > 0 ? feedStartS[feedStartS.length - 1] : 0.0)
                    + feedHoldS + endPaddingS;

            if (t < lastEnd) return true;

            if (feeder != null) feeder.setPosition(loadPos);
            if (shooter != null) shooter.setPower(0.0);
            return false;
        }
    }

    /** Legacy: shooter VELOCITY (TPS) + scheduled feed windows. */
    public static class ShooterAndFeederActionVel implements Action {
        private final DcMotorEx shooter;
        private final Servo feeder;
        private final double shooterVel;
        private final double[] feedStartS;
        private final double feedHoldS;
        private final double endPaddingS;
        private final double loadPos;
        private final double feedPos;

        private boolean initialized = false;
        private long t0;

        public ShooterAndFeederActionVel(
                DcMotorEx shooter,
                Servo feeder,
                double shooterVel,
                double[] feedStartS,
                double feedHoldS,
                double endPaddingS,
                double loadPos,
                double feedPos
        ) {
            this.shooter = shooter;
            this.feeder = feeder;
            this.shooterVel = shooterVel;
            this.feedStartS = (feedStartS != null) ? feedStartS : new double[0];
            this.feedHoldS = Math.max(0.0, feedHoldS);
            this.endPaddingS = Math.max(0.0, endPaddingS);
            this.loadPos = loadPos;
            this.feedPos = feedPos;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                t0 = System.nanoTime();
                if (shooter != null) {
                    shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    shooter.setVelocity(shooterVel);
                }
                if (feeder != null) feeder.setPosition(loadPos);
                initialized = true;
            }

            double t = (System.nanoTime() - t0) / 1e9;

            boolean feeding = false;
            for (double s : feedStartS) {
                if (t >= s && t < s + feedHoldS) { feeding = true; break; }
            }
            if (feeder != null) feeder.setPosition(feeding ? feedPos : loadPos);

            packet.put("t_s", String.format("%.2f", t));
            packet.put("feeding", feeding);

            double lastEnd = (feedStartS.length > 0 ? feedStartS[feedStartS.length - 1] : 0.0)
                    + feedHoldS + endPaddingS;

            if (t < lastEnd) return true;

            if (feeder != null) feeder.setPosition(loadPos);
            if (shooter != null) shooter.setVelocity(0.0);
            return false;
        }
    }


    public static class ShooterAndCRFeederActionVel implements Action {
        private final DcMotorEx shooter;
        private final CRServo feeder;
        private final double shooterVel;
        private final double waitTime;
        private long t0;
        private final double timeToShoot;
        private boolean init;
        private long prevAlignNs = 0L;



        public ShooterAndCRFeederActionVel(DcMotorEx shooter, CRServo feeder,
                                           double shooterVel, double waitTime,
                                           double timeToShoot) {
            this.shooter = shooter;
            this.feeder = feeder;
            this.shooterVel = shooterVel;
            this.waitTime = waitTime;
            this.timeToShoot = timeToShoot;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!init) {
                init = true;
                t0 = System.nanoTime();
                if (shooter != null) {
                    shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    shooter.setVelocity(shooterVel);
                }

                if (feeder != null) feeder.setPower(0);
            }

            double t = (System.nanoTime() - t0) / 1e9;
            if(t >= waitTime){
                feeder.setPower(-1);
            }
            if(t >= timeToShoot){
               if(feeder != null) feeder.setPower(0);
               if(shooter != null) shooter.setVelocity(0.0);
               return false;
            }
            return true;
        }
    }
    public static class ShooterAndFeederCombined implements Action {
        private final DcMotorEx shooter;
        private final DcMotor intake;
        private final CRServo feedServo;
        private final CRServo sideServo;
        private final DistanceSensor rangeSensor;
        private final double shooterVel;
        private final double sidePower;
        private final double waitTime;
        private long t0;
        private final double timeToShoot;
        private boolean init;



        public ShooterAndFeederCombined(DcMotorEx shooter, DcMotor intake, CRServo feedServo, CRServo sideServo,
                                                   DistanceSensor rangeSensor,
                                                   double shooterVel,double sidePower, double waitTime,
                                                   double timeToShoot) {
            this.shooter = shooter;
            this.intake = intake;
            this.feedServo = feedServo;
            this.sideServo = sideServo;
            this.rangeSensor =rangeSensor;
            this.shooterVel = shooterVel;
            this.sidePower =sidePower;
            this.waitTime = waitTime;
            this.timeToShoot = timeToShoot;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!init) {
                init = true;
                t0 = System.nanoTime();
                if (shooter != null) {
                    shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    shooter.setVelocity(shooterVel);
                }

                if (feedServo != null) feedServo.setPower(0);
            }

            double t = (System.nanoTime() - t0) / 1e9;
            if(t >= waitTime){
                feedServo.setPower(-1);
            }
            boolean nofirstball = false;
            double distance = rangeSensor.getDistance(DistanceUnit.MM);
            if (!nofirstball && distance > 127){
                 sideServo.setPower(sidePower);
                    intake.setPower(0.75);
                    nofirstball = true;
            }
            if(t >= timeToShoot){
                if(feedServo != null) feedServo.setPower(0);
                if(shooter != null) shooter.setVelocity(0.0);
                if(intake != null) intake.setPower(0.0);
                if(sideServo != null) sideServo.setPower(sidePower);
                return false;
            }
            return true;
        }
    }




    // -------------------------------------------------------------------------
    // Vision shooter action (renamed to avoid colliding with legacy ShooterAndFeederAction)
    // -------------------------------------------------------------------------

    /**
     * Limelight-based auto shooter action (distance->TPS table), gated firing:
     * - Reads AprilTag range from Limelight
     * - Uses ShooterConfig.lookupTpsFromDistanceIn(rangeIn)
     * - Waits until at-speed is stable, then pulses feeder for each shot
     */




    public static class ShooterAndCRFeederVisionAction implements Action {
        private static final double M_TO_IN = 39.37007874015748;

        private final DcMotorEx shooter;
        private final CRServo feeder;
        private final Limelight3A limelight;
        private final CRServo sideServo;
        private final MecanumDrive drive;

        private long prevAlignNs = 0L;
        private double prevAlignErr = 0.0;
        private Double distIn_filt = null;  // Initialize as null
        private Double txTarget = null;     // Initialize as null
        private long alignedSinceNs = 0L;   // Track how long we've been aligned

        private final int goalTagId;
        private final int shots;
        private final double feedHoldS;
        private final double endPaddingS;

        private boolean initialized = false;
        private long t0Ns = 0L;

        private int shotsFired = 0;
        private boolean feeding = false;
        private long feedStartNs = 0L;
        private long lastShotEndNs = 0L;

        private final double ShooterVelocity;

        private static final long BETWEEN_SHOTS_NS = 350_000_000L;
        private static final long RESULT_STALE_MS = 100;
        private static final long MAX_ACTION_NS = 4_500_000_000L;
        private static final long HOLD_LAST_GOOD_NS = 250_000_000L;
        private static final long AT_SPEED_STABLE_NS = 150_000_000L;
        private static final long ALIGNED_STABLE_NS = 100_000_000L; // 100ms aligned before shooting

        private double lastGoodTPS = 0.0;
        private long lastGoodNs = 0L;
        private long atSpeedSinceNs = 0L;

        public ShooterAndCRFeederVisionAction(
                DcMotorEx shooter,
                CRServo feeder,
                Limelight3A limelight,
                CRServo SideServo,
                MecanumDrive drive,
                int goalTagId,
                int shots,
                double feedHoldS,
                double endPaddingS,
                double shooterVelocity
        ) {
            this.shooter = shooter;
            this.feeder = feeder;
            this.limelight = limelight;
            this.sideServo = SideServo;
            this.drive = drive;
            this.goalTagId = goalTagId;
            this.shots = Math.max(0, shots);
            this.feedHoldS = Math.max(0.0, feedHoldS);
            this.endPaddingS = Math.max(0.0, endPaddingS);
            ShooterVelocity = shooterVelocity;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            long now = System.nanoTime();

            if (!initialized) {
                t0Ns = now;
                if (shooter != null) {
                    shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    shooter.setPower(0.0);
                }
                if (feeder != null) feeder.setPower(0.0);
                if (sideServo != null) sideServo.setPower(0.0);
                if (drive != null) drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                initialized = true;
            }

            // Hard timeout
            double time = (now - t0Ns) / 1e9;

            if ((now - t0Ns) > MAX_ACTION_NS) {
                finish();
                return false;
            }


            // Get vision data
            Double tx = getVisionTx();


            // Command shooter
            if (shooter != null) shooter.setVelocity(ShooterVelocity);


            // AUTO-ALIGNMENT
            boolean aligned = false;
            double turnPower = 0.0;

            if (ShooterConfig.AUTO_ALIGN_ENABLED && distIn_filt != null && tx != null) {
                // Lookup allowed Tx window
                double[] window = ShooterConfig.lookupTxWindowFromDistanceIn(distIn_filt);
                double txMin = window[0];
                double txMax = window[1];

                // Determine target Tx
                if (txTarget == null || tx < txMin || tx > txMax) {
                    // First time or outside window - aim for center
                    txTarget = (txMin + txMax) / 2.0;
                }
                // Otherwise keep current target (we're in window)

                // Calculate error (positive error = need to turn right/positive)
                double error = tx - txTarget;

                // Check if aligned (within window AND small error)
                boolean inWindow = (tx >= txMin && tx <= txMax);
                boolean smallError = Math.abs(error) <= ShooterConfig.ALIGN_ERR_DEADBAND_DEG;
                aligned = inWindow && smallError;

                // Track stability
                if (aligned) {
                    if (alignedSinceNs == 0L) alignedSinceNs = now;
                } else {
                    alignedSinceNs = 0L;
                }

                // Compute turn power (only if not feeding)
                if (!feeding) {
                    turnPower = computeAlignTurnFromErr(error, now);
                }

                // Telemetry
                packet.put("align_tx", String.format("%.2f", tx));
                packet.put("align_target", String.format("%.2f", txTarget));
                packet.put("align_window", String.format("[%.2f, %.2f]", txMin, txMax));
                packet.put("align_error", String.format("%.2f", error));
                packet.put("align_in_window", inWindow);
                packet.put("aligned", aligned);
            } else {
                // No alignment possible
                aligned = !ShooterConfig.AUTO_ALIGN_ENABLED; // If disabled, always "aligned"
                packet.put("align_status", "no_vision_or_disabled");
            }

            boolean alignedStable = aligned && alignedSinceNs != 0L &&
                    (now - alignedSinceNs) >= ALIGNED_STABLE_NS;

            // Command drive
            if (drive != null) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), turnPower));
            }

            // If currently feeding, wait out the hold time
            if (feeding) {
                long holdNs = (long) (feedHoldS * 1e9);
                if ((now - feedStartNs) >= holdNs) {
                    if (feeder != null) feeder.setPower(0.0);
                    if (sideServo != null) sideServo.setPower(0.0);
                    feeding = false;
                    lastShotEndNs = now;
                    shotsFired++;
                    atSpeedSinceNs = 0L; // re-stabilize after shot
                    alignedSinceNs = 0L; // re-align after shot
                }
                return true;
            }

            // Done firing?


            // Spacing + gating
            boolean spacingOk = (lastShotEndNs == 0L) ||
                    ((now - lastShotEndNs) >= BETWEEN_SHOTS_NS);
            boolean canFire = ShooterVelocity > 0.0;

            if (canFire) {
                if (feeder != null) feeder.setPower(-1);
                if (sideServo != null) sideServo.setPower(-1);
                feeding = true;
                feedStartNs = now;
            }

            return true;
        }

        private void finish() {
            if (feeder != null) feeder.setPower(0.0);
            if (sideServo != null) sideServo.setPower(0.0);
            if (shooter != null) shooter.setPower(0.0);
            if (drive != null) drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            feeding = false;
            atSpeedSinceNs = 0L;
            alignedSinceNs = 0L;
        }


        private Double getVisionTx() {
            if (limelight == null) return null;
            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid() ||
                    result.getStaleness() >= RESULT_STALE_MS) return null;

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials == null) return null;

            for (LLResultTypes.FiducialResult f : fiducials) {
                if (f == null || f.getFiducialId() != goalTagId) continue;
                return f.getTargetXDegrees();
            }
            return null;
        }

        private double computeAlignTurnFromErr(double errDeg, long now) {
            double dt = (prevAlignNs == 0L) ? 0.02 : (now - prevAlignNs) / 1e9;
            if (dt > 0.5) dt = 0.02; // Guard against huge dt
            prevAlignNs = now;

            double derr = (errDeg - prevAlignErr) / dt;
            prevAlignErr = errDeg;

            double u = ShooterConfig.ALIGN_KP * errDeg + ShooterConfig.ALIGN_KD * derr;

            // Clamp to max
            if (u > ShooterConfig.ALIGN_MAX_TURN) u = ShooterConfig.ALIGN_MAX_TURN;
            if (u < -ShooterConfig.ALIGN_MAX_TURN) u = -ShooterConfig.ALIGN_MAX_TURN;

            // Apply minimum turn (overcome static friction)
            if (Math.abs(u) > 0.0 && Math.abs(u) < ShooterConfig.ALIGN_MIN_TURN) {
                u = Math.copySign(ShooterConfig.ALIGN_MIN_TURN, u);
            }

            return u;
        }
    }



    public static class ShooterAndFeederVisionAction implements Action {
        private static final double M_TO_IN = 39.37007874015748;

        private final DcMotorEx shooter;
        private final CRServo feeder;
        private final Limelight3A limelight;

        private final int goalTagId;
        private final int shots;
        private final double loadPos;
        private final double feedPos;
        private final double feedHoldS;
        private final double endPaddingS;

        private boolean initialized = false;
        private long t0Ns = 0L;

        private int shotsFired = 0;
        private boolean feeding = false;
        private long feedStartNs = 0L;
        private long lastShotEndNs = 0L;

        private double shooterSetpointTPS = 0.0;

        private static final long BETWEEN_SHOTS_NS = 350_000_000L;
        private static final long RESULT_STALE_MS = 100;

        private static final long MAX_ACTION_NS = 4_500_000_000L;
        private static final long HOLD_LAST_GOOD_NS = 250_000_000L;
        private static final long AT_SPEED_STABLE_NS = 150_000_000L;

        private double lastGoodTPS = 0.0;
        private long lastGoodNs = 0L;

        private long atSpeedSinceNs = 0L;

        public ShooterAndFeederVisionAction(
                DcMotorEx shooter,
                CRServo feeder,
                Limelight3A limelight,
                int goalTagId,
                int shots,
                double loadPos,
                double feedPos,
                double feedHoldS,
                double endPaddingS
        ) {
            this.shooter = shooter;
            this.feeder = feeder;
            this.limelight = limelight;
            this.goalTagId = goalTagId;
            this.shots = Math.max(0, shots);
            this.loadPos = loadPos;
            this.feedPos = feedPos;
            this.feedHoldS = Math.max(0.0, feedHoldS);
            this.endPaddingS = Math.max(0.0, endPaddingS);
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            long now = System.nanoTime();

            if (!initialized) {
                t0Ns = now;

                if (shooter != null) {
                    shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    shooter.setPower(0.0);
                }
                if (feeder != null) feeder.setPower(0.0);

                initialized = true;
            }

            // Hard timeout so we don't hang auto forever if vision fails
            if ((now - t0Ns) > MAX_ACTION_NS) {
                finish();
                return false;
            }

            shooterSetpointTPS = 0.0;

            // Get range (in) from Limelight
            Double rangeIn = getVisionDistanceInches(); // sqrt(x^2+z^2) using cameraPoseTargetSpace
            if (rangeIn != null && rangeIn >= ShooterConfig.MIN_RANGE_IN) {

                double desiredTps = ShooterConfig.lookupTpsFromDistanceIn(rangeIn);

                if (desiredTps >= ShooterConfig.TPS_MIN_AUTO) {
                    desiredTps = Math.min(desiredTps, ShooterConfig.TPS_MAX_AUTO);
                    desiredTps = Math.min(desiredTps, ShooterConfig.TPS_MAX_MECH);

                    shooterSetpointTPS = desiredTps;
                    lastGoodTPS = desiredTps;
                    lastGoodNs = now;
                }
            }

            // Hold last good for a short window if tag drops
            if (shooterSetpointTPS <= 0.0) {
                if (lastGoodTPS > 0.0 && (now - lastGoodNs) <= HOLD_LAST_GOOD_NS) {
                    shooterSetpointTPS = lastGoodTPS;
                }
            }

            // Command shooter
            if (shooter != null) {
                if (shooterSetpointTPS > 0.0) shooter.setVelocity(shooterSetpointTPS);
                else shooter.setPower(0.0);
            }

            // At-speed check
            boolean atSpeed = false;
            double actual = 0.0;
            if (shooter != null && shooterSetpointTPS > 0.0) {
                actual = shooter.getVelocity();
                atSpeed = Math.abs(actual - shooterSetpointTPS) <= ShooterConfig.TPS_TOL;
            }

            if (atSpeed) {
                if (atSpeedSinceNs == 0L) atSpeedSinceNs = now;
            } else {
                atSpeedSinceNs = 0L;
            }

            boolean atSpeedStable = atSpeed && atSpeedSinceNs != 0L && (now - atSpeedSinceNs) >= AT_SPEED_STABLE_NS;

            // Telemetry for debugging
            packet.put("vision_range_in", rangeIn == null ? "null" : String.format("%.1f", rangeIn));
            packet.put("shooter_set_tps", String.format("%.0f", shooterSetpointTPS));
            packet.put("shooter_actual_tps", String.format("%.0f", actual));
            packet.put("atSpeed", atSpeed);
            packet.put("atSpeedStable", atSpeedStable);
            packet.put("shotsFired", shotsFired);

            // If currently feeding, wait out the hold time
            if (feeding) {
                long holdNs = (long) (feedHoldS * 1e9);
                if ((now - feedStartNs) >= holdNs) {
                    if (feeder != null) feeder.setPower(0.0);
                    feeding = false;
                    lastShotEndNs = now;
                    shotsFired++;
                    atSpeedSinceNs = 0L; // re-stabilize after a shot
                }
                return true;
            }

            // Done firing?
            if (shotsFired >= shots) {
                if (lastShotEndNs != 0L) {
                    long endPadNs = (long) (endPaddingS * 1e9);
                    if ((now - lastShotEndNs) >= endPadNs) {
                        finish();
                        return false;
                    }
                    return true;
                } else {
                    finish();
                    return false;
                }
            }

            // Spacing + gating
            boolean spacingOk = (lastShotEndNs == 0L) || ((now - lastShotEndNs) >= BETWEEN_SHOTS_NS);
            boolean canFire = atSpeedStable && spacingOk && shooterSetpointTPS > 0.0;

            if (canFire) {
                if (feeder != null) feeder.setPower(-1);
                feeding = true;
                feedStartNs = now;
            }

            return true;
        }

        private void finish() {
            if (feeder != null) feeder.setPower(0.0);
            if (shooter != null) shooter.setPower(0.0);
            shooterSetpointTPS = 0.0;
            feeding = false;
            atSpeedSinceNs = 0L;
        }

        /**
         * distance = sqrt(x^2 + z^2) in inches, using cameraPoseTargetSpace
         */
        private Double getVisionDistanceInches() {
            if (limelight == null) return null;

            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid() || result.getStaleness() >= RESULT_STALE_MS) return null;

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials == null) return null;

            for (LLResultTypes.FiducialResult f : fiducials) {
                if (f == null) continue;
                if (f.getFiducialId() != goalTagId) continue;

                Pose3D pose = f.getCameraPoseTargetSpace();
                if (pose == null) continue;

                double xM = pose.getPosition().x;
                double zM = pose.getPosition().z;

                double rangeM = Math.sqrt((xM * xM) + (zM * zM));
                return rangeM * M_TO_IN;
            }

            return null;
        }
    }
}