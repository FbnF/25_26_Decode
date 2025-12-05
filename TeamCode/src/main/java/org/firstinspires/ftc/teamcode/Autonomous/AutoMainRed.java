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
@Autonomous(name="AutoMainRedSide", group="Main")

public class AutoMainRed extends LinearOpMode {

    // --- HELPER METHODS DEFINED AT CLASS LEVEL (OUTSIDE runOpMode) ---

    // 1. Launch Action: Runs motor for a duration, then stops (self-completing).
    private Action launchForDuration(DcMotor m, double p, double seconds) {
        return new Action() {
            private boolean initialized = false;
            private long startTimeNanos;
            private final long durationNanos = (long) (seconds * 1_000_000_000L);

            @Override
            public boolean run(TelemetryPacket packet) {
                Servo feedServo = hardwareMap.get(Servo.class, "FeedServo");
                if (!initialized) {

                    startTimeNanos = System.nanoTime();
                    telemetry.addData("Current time",System.nanoTime( ));
                    telemetry.update();
                    initialized = true;
                    while (System.nanoTime() - startTimeNanos < durationNanos){
                        LoadServo(feedServo, true);
                        m.setPower(Math.abs(p));
                        LoadServo(feedServo, false);
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

    private Action LoadServo (Servo m, boolean isLoaded) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!initialized) {
                    if(isLoaded){
                        m.setPosition(0.75);
                    } else {
                        m.setPosition(0);
                    }
                    telemetry.addData("Current time",System.nanoTime( ));
                    telemetry.update();
                    initialized = true;
                    //return false;
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
        Pose2d startPose = new Pose2d(60, 16, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Motors you want to toggle during "waits"
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        DcMotor launchMotor = hardwareMap.get(DcMotorEx.class, "LaunchMotor");
        Servo feedServo = hardwareMap.get(Servo.class, "FeedServo");
        // unused here, just leaving as-is

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor.setPower(0.0);
        launchMotor.setPower(0.0);

        Action all = drive.actionBuilder(startPose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d( 0, 0, Math.toRadians(135)), Math.PI / 2)
                .turn(Math.toRadians(-45))
                //first artifact round
                .setTangent(0)
                .lineToX(-10)
                .setTangent(90)
                .strafeTo(new Vector2d(-10, 52))
                //second artifact round
                .setTangent(0)
                .splineToLinearHeading(new Pose2d( 0, 0, Math.toRadians(135)), Math.PI / 2)
                .turn(Math.toRadians(-45))
                .setTangent(0)
                .lineToX(10)
                .setTangent(90)
                .strafeTo(new Vector2d(10, 52))
                //third artifact round
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(135)), Math.PI / 2)
                .turn(Math.toRadians(-45))
                .setTangent(0)
                .lineToX(34)
                .setTangent(90)
                .strafeTo(new Vector2d(34, 52))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(135)), Math.PI / 2)


                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(all);

        // safety
        intakeMotor.setPower(0);
        if (launchMotor != null) launchMotor.setPower(0);
    }
}