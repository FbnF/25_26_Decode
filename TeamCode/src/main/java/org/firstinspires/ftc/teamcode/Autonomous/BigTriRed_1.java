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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Disabled
@Autonomous(name="MEET1: BigTriRed", group="Main")
public class BigTriRed_1 extends LinearOpMode {

    // --- HELPER METHODS DEFINED AT CLASS LEVEL (OUTSIDE runOpMode) ---

    // 1. Launch Action: Runs motor for a duration, then stops (self-completing).
    private Action launchForDuration(DcMotor m, double p, double seconds) {
        return new Action() {
            private boolean initialized = false;
            private long startTimeNanos;
            private final long durationNanos = (long) (seconds * 1_000_000_000L);
            Servo feedServo = hardwareMap.get(Servo.class, "feedServo");

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!initialized) {

                    startTimeNanos = System.nanoTime();
                    telemetry.addData("Current time", System.nanoTime());
                    telemetry.update();
                    initialized = true;
                    while (System.nanoTime() - startTimeNanos < durationNanos) {
                        m.setPower(Math.abs(p));
                        for (int i = 0; i <100; i++){

                            if (i == 30 || i == 63 || i==96){
                                LaunchServo(feedServo, true);
                            }
                        }
                    }
                    //return false;
                }
                if (System.nanoTime() - startTimeNanos >= durationNanos) {
                    m.setPower(0.0);
                    // return true; // Action complete
                }
                return false;
            }

            public void preview(TelemetryPacket packet) {
            }
        };
    }
    private Action LaunchServo(Servo s, Boolean isActive) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!initialized) {
                    if (isActive){
                        s.setPosition(0.75);
                    }else {
                        s.setPosition(0);
                    }

                    //return false;
                }
                return false;
            }

            public void preview(TelemetryPacket packet) {
            }
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
        Pose2d startPose = new Pose2d(-60, 34, Math.toRadians(90));
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

                .lineToY(12)
                .stopAndAdd(launchForDuration(launchMotor, 0.55, 6))
                .strafeTo(new Vector2d(12,-24))
                .turn(Math.toRadians(180))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(all);

        // safety
        intakeMotor.setPower(0);
        if (launchMotor != null) launchMotor.setPower(0);
    }
}