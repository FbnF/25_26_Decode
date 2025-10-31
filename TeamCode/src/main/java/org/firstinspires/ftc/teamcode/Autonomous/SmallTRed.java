/*package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="AutoMainSmallTRED", group="Main")
public class SmallTRed extends LinearOpMode {

    // --- HELPER METHODS DEFINED AT CLASS LEVEL (OUTSIDE runOpMode) ---

    // 1. Launch Action: Runs motor for a duration, then stops (self-completing).
    private Action launchForDuration(DcMotor m, double p, double seconds) {
        return new Action() {
            private boolean initialized = false;
            private long startTimeNanos;
            private final long durationNanos = (long) (seconds * 1_000_000_000L);

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!initialized) {

                    startTimeNanos = System.nanoTime();
                    telemetry.addData("Current time",System.nanoTime( ));
                    telemetry.update();
                    initialized = true;
                    while (System.nanoTime() - startTimeNanos < durationNanos){
                        m.setPower(Math.abs(p));
                    }
                    //return false;
                }
                if (System.nanoTime() - startTimeNanos >= durationNanos) {
                    m.setPower(0.0);
                   // return true; // Action complete
                }
                return false;
            }
            public void preview(TelemetryPacket packet) {}
        };
    }

    // 2. Intake Action: Runs motor continuously/in parallel (returns false).
    private Action motorRun(DcMotor m, double p) {
        return (TelemetryPacket packet) -> {
            m.setPower(Math.abs(p));
            return false; // RUNNING: Keeps the motor on while the trajectory executes
        };
    }

    // 3. Stop Action: Stops a motor (returns true).
    private Action motorStop(DcMotor m) {
        return (TelemetryPacket packet) -> {
            m.setPower(0.0);
            return true; // COMPLETE: Stops the motor
        };
    }

    // -------------------------------------------------------------------


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(60, 24, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Motors you want to toggle during "waits"
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        DcMotor launchMotor = hardwareMap.get(DcMotorEx.class, "LaunchMotor"); // unused here, just leaving as-is

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor.setPower(0.0);
        launchMotor.setPower(0.0);

        Action all = drive.actionBuilder(startPose)
                // --- Leg 1 ---
                .splineTo(new Vector2d(-12, 12), Math.toRadians(135))
                .stopAndAdd(launchForDuration(launchMotor,0.65,2))
                .turn(Math.toRadians(-60))
                //.splineToLinearHeading(new Pose2d(-4, 32,Math.toRadians(110)),Math.toRadians(110))
                .lineToY(48)
                .lineToY(12)
                .turn(Math.toRadians(60))
                .stopAndAdd(launchForDuration(launchMotor,0.65,2))
                .turn(Math.toRadians(-60))
                .lineToX(16)
                .lineToY(48)
                .turn(Math.toRadians(-60))
                .setTangent(0)
                .lineToX(16)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(0)
                .lineToY(48)
                .setTangent(Math.toRadians(90))
                .lineToX(16)
                .turn(Math.toRadians(-60))
                .stopAndAdd(launchForDuration(launchMotor,0.65,2))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(all);

        // safety
        intakeMotor.setPower(0);
        if (launchMotor != null) launchMotor.setPower(0);
    }
}*/


package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.MainCode.util.SafeHardware;

@Disabled
@Autonomous(name="AutoMainSmallTRED", group="Simple")
public class SmallTRed extends LinearOpMode {

    // ---- Names in RC config ----
    private static final String INTAKE_NAME = "IntakeMotor";
    private static final String SHOOTER_NAME = "LaunchMotor";

    // ==== Non-blocking helpers (RR: true=keep running, false=done) ====

    /** Non-blocking wait. */
    private static Action waitSeconds(double seconds) {
        return new Action() {
            private boolean inited = false;
            private long t0;
            @Override public boolean run(TelemetryPacket p) {
                if (!inited) { t0 = System.nanoTime(); inited = true; }
                double t = (System.nanoTime() - t0) / 1e9;
                return t < seconds; // keep running until time elapses
            }
        };
    }

    /** NON-BLOCKING, NULL-SAFE: run motor at p for `seconds`, then stop. */
    private Action launchForDuration(DcMotor m, double p, double seconds) {
        return new Action() {
            private boolean inited = false;
            private long t0;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!inited) {
                    t0 = System.nanoTime();
                    if (m != null) {
                        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        m.setPower(Math.abs(p));
                    }
                    inited = true;
                }

                double t = (System.nanoTime() - t0) / 1e9;
                packet.put("launch_t", String.format("%.2f", t));

                if (t < seconds) {
                    return true; // keep running
                }

                // finished: stop if present
                if (m != null) m.setPower(0.0);
                return false; // done
            }

            public void preview(TelemetryPacket packet) {}
        };
    }

    /** Run motor continuously (null-safe). Finishes immediately (non-blocking one-shot). */
    private Action motorRun(DcMotor m, double p) {
        return (TelemetryPacket packet) -> {
            if (m != null) {
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m.setPower(Math.abs(p));
            }
            return false; // one-shot, follower continues
        };
    }

    /** Stop motor (null-safe). Completes immediately. */
    private Action motorStop(DcMotor m) {
        return (TelemetryPacket packet) -> {
            if (m != null) m.setPower(0.0);
            return false;
        };
    }

    // -------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {
        // Try to create drive (this may fail if base motors aren't in config)
        Pose2d startPose = new Pose2d(60, 24, Math.toRadians(180));
        MecanumDrive drive = null;
        boolean driveAvailable = true;
        try {
            drive = new MecanumDrive(hardwareMap, startPose);
        } catch (Exception e) {
            driveAvailable = false;
            telemetry.addLine("⚠️ Drive not available: " + e.getClass().getSimpleName());
            telemetry.addLine("   Running fallback (no path).");
            telemetry.update();
        }

        // Safe hardware (null if missing)
        DcMotor intakeMotor  = SafeHardware.tryDcMotor(hardwareMap, INTAKE_NAME);
        DcMotorEx launchMotor = SafeHardware.tryDcMotorEx(hardwareMap, SHOOTER_NAME);

        // Safe setup (only touch if present)
        if (intakeMotor != null) {
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setPower(0.0);
        } else {
            telemetry.addLine("⚠️ IntakeMotor missing.");
        }

        if (launchMotor != null) {
            launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launchMotor.setPower(0.0);
        } else {
            telemetry.addLine("⚠️ LaunchMotor missing.");
        }
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        if (driveAvailable) {
            // === Normal RR path ===
            Action all = drive.actionBuilder(startPose)
                    // Start intake (non-blocking one-shot) so it runs while driving
                    .stopAndAdd(motorRun(intakeMotor, 0.6))

                    // --- Leg 1 ---
                    .splineTo(new Vector2d(-12, 12), Math.toRadians(135))

                    // Shooter run for 2s (non-blocking action, base pauses here)
                    .stopAndAdd(launchForDuration(launchMotor, 0.65, 2.0))

                    .turn(Math.toRadians(-60))
                    // .splineToLinearHeading(new Pose2d(-4, 32,Math.toRadians(110)),Math.toRadians(110))
                    .lineToY(48)
                    .lineToY(12)
                    .turn(Math.toRadians(60))

                    // Second shooter run
                    .stopAndAdd(launchForDuration(launchMotor, 0.65, 2.0))

                    .turn(Math.toRadians(-60))
                    .lineToX(16)
                    .lineToY(48)
                    .turn(Math.toRadians(-60))
                    .setTangent(0)
                    .lineToX(16)
                    .setTangent(Math.toRadians(90))
                    .lineToY(48)
                    .setTangent(0)
                    .lineToY(48)
                    .setTangent(Math.toRadians(90))
                    .lineToX(16)
                    .turn(Math.toRadians(-60))

                    // Third shooter run
                    .stopAndAdd(launchForDuration(launchMotor, 0.65, 2.0))

                    // Stop intake (one-shot)
                    .stopAndAdd(motorStop(intakeMotor))

                    .build();

            Actions.runBlocking(all);
        } else {
            // === Fallback (no drive): just exercise the shooter timing so OpMode can run on bench ===
            Actions.runBlocking(
                    // start intake (if present), wait, shoot for 2s, wait, stop intake
                    motorRun(intakeMotor, 0.6)
            );
            Actions.runBlocking(waitSeconds(1.0));
            Actions.runBlocking(launchForDuration(launchMotor, 0.65, 2.0));
            Actions.runBlocking(waitSeconds(0.5));
            Actions.runBlocking(motorStop(intakeMotor));
        }

        // safety
        if (intakeMotor != null) intakeMotor.setPower(0.0);
        if (launchMotor != null) launchMotor.setPower(0.0);
    }
}