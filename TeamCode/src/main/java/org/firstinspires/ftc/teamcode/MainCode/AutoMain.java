package org.firstinspires.ftc.teamcode.MainCode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket; // if you have the dashboard dep
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="AutoMain BlueSide", group="AutoMain")
public class AutoMain extends LinearOpMode {
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
    // Small helper Action that sets a motor power once and immediately completes
    private static Action motorPower(DcMotor m, double p) {
        m.setPower(p);
        return (TelemetryPacket packet) -> { m.setPower(Math.abs(p)); return true; };
    }
    public  class  MotorLaunch implements Action{
        DcMotor motor;
        Double power;

        public MotorLaunch(DcMotor launchMotor, double v) {

            this.motor = motor;
            motor.setPower(power);
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            motor.setPower(power);
            return false;
        }
    }

    private static Action servoPower(Servo m, boolean isArmed){
        if(isArmed){
            return (TelemetryPacket packet) -> { m.setPosition(0); return true; };
        } else {
            return (TelemetryPacket packet) -> { m.setPosition(0.75); return true; };
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(60, 16, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Motors you want to toggle during "waits"
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        DcMotor launchMotor = hardwareMap.get(DcMotor.class, "LaunchMotor");
        Servo   feedServo = hardwareMap.get(Servo.class, "feedServo");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor.setPower(0.0);
        launchMotor.setPower(0.0);

        Action all = drive.actionBuilder(startPose)
                // --- Leg 1 ---
             /*   .splineTo(new Vector2d(15, -10), Math.toRadians(135))

                .stopAndAdd(motorPower(launchMotor, 1.0))   // ON
                .waitSeconds(2)
                .stopAndAdd(motorPower(launchMotor, 0.0))   // OFF

                .setTangent(Math.toRadians(90))
                .afterDisp(0.0, motorPower(intakeMotor, 1.0))
                .splineToLinearHeading(new Pose2d(-11, 33, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(45)
                .lineToY(33)
                .afterDisp(0.0, motorPower(intakeMotor, 0.0))

                // --- Leg 2 ---
                .splineTo(new Vector2d(0, 0), Math.toRadians(-40))
                .stopAndAdd(motorPower(launchMotor, 1.0))   // ON
                .waitSeconds(2)
                .stopAndAdd(motorPower(launchMotor, 0.0))   // OFF

                .setTangent(Math.toRadians(90))
                .afterDisp(0.0, motorPower(intakeMotor, 1.0))
                .splineToLinearHeading(new Pose2d(11, 33, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(45)
                .lineToY(33)
                .afterDisp(0.0, motorPower(intakeMotor, 0.0))

                // --- Leg 3 ---
                .splineTo(new Vector2d(0, 0), Math.toRadians(-40))
                .stopAndAdd(motorPower(launchMotor, 1.0))   // ON
                .waitSeconds(2)
                .stopAndAdd(motorPower(launchMotor, 0.0))   // OFF

                .setTangent(Math.toRadians(90))
                .afterDisp(0.0, motorPower(intakeMotor, 1.0))
                .splineToLinearHeading(new Pose2d(34, 33, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(45)
                .lineToY(33)
                .afterDisp(0.0, motorPower(intakeMotor, 0.0))

                // --- Leg 4 ---
                .splineTo(new Vector2d(0, 0), Math.toRadians(-40))
                .stopAndAdd(motorPower(launchMotor, 1.0))   // ON
                .waitSeconds(2)
                .stopAndAdd(motorPower(launchMotor, 0.0)) */
                // OFF
                .setTangent(0)
                .splineToLinearHeading(new Pose2d( 0, 0, Math.toRadians(225)), Math.PI / 2)
                .stopAndAdd(launchForDuration(launchMotor, 0.64, 2))
                //.stopAndAdd(servoPower(feedServo, false))
                //.stopAndAdd(motorPower(launchMotor, 0.63345))
                //Actions.runBlocking(motorPower(launchMotor, 0.65))
                //This is where the intake motor would run
                .splineToLinearHeading(new Pose2d( -14, -52, Math.toRadians(225)), Math.PI / 2)
                .splineToLinearHeading(new Pose2d( 0, 0, Math.toRadians(225)), Math.PI / 2)
                //launch balls
                .splineToLinearHeading(new Pose2d( 14, -52, Math.toRadians(270)), Math.PI / 2)
                //Run intake
                .splineToLinearHeading(new Pose2d( 0, 0, Math.toRadians(225)), Math.PI / 2)
                //Launch balls again
                .stopAndAdd(launchForDuration(launchMotor, 0.64, 2))
                .splineToLinearHeading(new Pose2d( 38, -52, Math.toRadians(270)), Math.PI / 2)
                //run the intake
                .splineToLinearHeading(new Pose2d( 0, 0, Math.toRadians(225)), Math.PI / 2)
                .stopAndAdd(launchForDuration(intakeMotor, 0.64, 2))

                //launch the balls
                //.stopAndAdd(servoPower(feedServo, true))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(all);

        // safety
        intakeMotor.setPower(0);
        if (launchMotor != null) launchMotor.setPower(0);
    }
}