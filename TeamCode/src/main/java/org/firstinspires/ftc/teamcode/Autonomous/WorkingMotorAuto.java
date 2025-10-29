package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Action;

@Autonomous(name="AutoMain", group="Main")
public class WorkingMotorAuto extends LinearOpMode {

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
                    m.setPower(Math.abs(p));
                    startTimeNanos = System.nanoTime();
                    initialized = true;
                    return false;
                }
                if (System.nanoTime() - startTimeNanos >= durationNanos) {
                    m.setPower(0.0);
                    return true; // Action complete
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
        Pose2d startPose = new Pose2d(60, 22, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Motors you want to toggle during "waits"
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        DcMotor launchMotor = hardwareMap.get(DcMotor.class, "LaunchMotor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0.0);
        launchMotor.setPower(0.0);

        // Create specific, reusable actions by calling the class-level helpers
        Action launch2s = launchForDuration(launchMotor, 1.0, 5.0);
        Action intakeOn  = motorRun(intakeMotor, 1.0);
        Action intakeOff = motorStop(intakeMotor);


        Action all = drive.actionBuilder(startPose)
                // --- Leg 1 ---
                .splineTo(new Vector2d(15, -10), Math.toRadians(135))

                // LAUNCH SEQUENCE: Stop, run for 2s, continue
                .stopAndAdd(launch2s)

                .setTangent(Math.toRadians(90))
                // INTAKE SEQUENCE: Start intake and run in parallel
                .afterDisp(0.0, intakeOn)
                .splineToLinearHeading(new Pose2d(-11, 33, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(45)
                .lineToY(33)
                // Stop intake
                .afterDisp(0.0, intakeOff)

                // --- Leg 2 ---
                .splineTo(new Vector2d(0, 0), Math.toRadians(-40))
                .stopAndAdd(launch2s)

                .setTangent(Math.toRadians(90))
                .afterDisp(0.0, intakeOn)
                .splineToLinearHeading(new Pose2d(11, 33, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(45)
                .lineToY(33)
                .afterDisp(0.0, intakeOff)

                // --- Leg 3 ---
                .splineTo(new Vector2d(0, 0), Math.toRadians(-40))
                .stopAndAdd(launch2s)

                .setTangent(Math.toRadians(90))
                .afterDisp(0.0, intakeOn)
                .splineToLinearHeading(new Pose2d(34, 33, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(45)
                .lineToY(33)
                .afterDisp(0.0, intakeOff)

                // --- Leg 4 ---
                .splineTo(new Vector2d(0, 0), Math.toRadians(-40))
                .stopAndAdd(launch2s)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(all);

        // safety
        intakeMotor.setPower(0);
        if (launchMotor != null) launchMotor.setPower(0);
    }
}