package org.firstinspires.ftc.teamcode.MainCode.util;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MainCode.config.ShooterConfig;

import java.util.List;

public final class AutoMotorControl {

    private AutoMotorControl() {}

    public static Action setMotorPower(DcMotor m, double power) {
        return (TelemetryPacket pkt) -> {
            if (m != null) {
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m.setPower(power);
            }
            return false;
        };
    }

    public static Action setMotorVel(DcMotorEx m, double tps) {
        return (TelemetryPacket pkt) -> {
            if (m != null) {
                m.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                m.setVelocity(tps);
            }
            return false;
        };
    }

    public static Action setServoPosition(Servo s, double pos) {
        return (TelemetryPacket pkt) -> {
            if (s != null) s.setPosition(pos);
            return false;
        };
    }

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
            this.feedHoldS = Math.max(0, feedHoldS);
            this.endPaddingS = Math.max(0, endPaddingS);
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

    // NOTE: Your Limelight-based auto shooter action (kept same, only distance changed)
    public static class ShooterAndFeederAction implements Action {
        private static final double M_TO_IN = 39.37007874015748;

        private final DcMotorEx shooter;
        private final Servo feeder;
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

        public ShooterAndFeederAction(
                DcMotorEx shooter,
                Servo feeder,
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
                if (feeder != null) feeder.setPosition(loadPos);

                initialized = true;
            }

            if ((now - t0Ns) > MAX_ACTION_NS) {
                finish();
                return false;
            }

            shooterSetpointTPS = 0.0;

            Double rangeIn = getVisionDistanceInches(); // now uses abs(z)
            if (rangeIn != null && rangeIn >= ShooterConfig.MIN_RANGE_IN) {

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

                if (Double.isFinite(tps) && !Double.isNaN(tps)) {
                    if (tps >= ShooterConfig.TPS_MIN_AUTO) {
                        tps = Math.min(tps, ShooterConfig.TPS_MAX_AUTO);
                        tps = Math.min(tps, ShooterConfig.TPS_MAX_MECH);
                        shooterSetpointTPS = tps;
                        lastGoodTPS = tps;
                        lastGoodNs = now;
                    }
                }
            }

            if (shooterSetpointTPS <= 0.0) {
                if (lastGoodTPS > 0.0 && (now - lastGoodNs) <= HOLD_LAST_GOOD_NS) {
                    shooterSetpointTPS = lastGoodTPS;
                }
            }

            if (shooter != null) {
                if (shooterSetpointTPS > 0.0) shooter.setVelocity(shooterSetpointTPS);
                else shooter.setPower(0.0);
            }

            boolean atSpeed = false;
            if (shooter != null && shooterSetpointTPS > 0.0) {
                double actual = shooter.getVelocity();
                atSpeed = Math.abs(actual - shooterSetpointTPS) <= ShooterConfig.TPS_TOL;
            }

            if (atSpeed) {
                if (atSpeedSinceNs == 0L) atSpeedSinceNs = now;
            } else {
                atSpeedSinceNs = 0L;
            }

            boolean atSpeedStable = atSpeed && atSpeedSinceNs != 0L && (now - atSpeedSinceNs) >= AT_SPEED_STABLE_NS;

            if (feeding) {
                long holdNs = (long) (feedHoldS * 1e9);
                if ((now - feedStartNs) >= holdNs) {
                    if (feeder != null) feeder.setPosition(loadPos);
                    feeding = false;
                    lastShotEndNs = now;
                    shotsFired++;
                    atSpeedSinceNs = 0L;
                }
                return true;
            }

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

            boolean spacingOk = (lastShotEndNs == 0L) || ((now - lastShotEndNs) >= BETWEEN_SHOTS_NS);
            boolean canFire = atSpeedStable && spacingOk && shooterSetpointTPS > 0.0;

            if (canFire) {
                if (feeder != null) feeder.setPosition(feedPos);
                feeding = true;
                feedStartNs = now;
            }

            return true;
        }

        private void finish() {
            if (feeder != null) feeder.setPosition(loadPos);
            if (shooter != null) shooter.setPower(0.0);
            shooterSetpointTPS = 0.0;
            feeding = false;
            atSpeedSinceNs = 0L;
        }

        /**
         * distance = abs(z) in inches.
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

                Pose3D pose = f.getRobotPoseTargetSpace();
                if (pose == null) continue;

                double zM = pose.getPosition().z;
                return Math.abs(zM) * M_TO_IN;
            }

            return null;
        }
    }
}