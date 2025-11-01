package org.firstinspires.ftc.teamcode.Autonomous;

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
import org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.ShooterAndFeederAction;

@Disabled
@Autonomous(name = "Auto: ExampleAuto", group = "Auto")
public class ExampleAuto extends LinearOpMode {

    // RC config names (same as BigTriBlue)
    private static final String FEED_SERVO   = "feedServo";
    private static final String INTAKE_MOTOR = "IntakeMotor";
    private static final String LAUNCH_MOTOR = "LaunchMotor";

    // Tunables
    private static final double INTAKE_POWER  = 1.0;
    private static final double SHOOTER_POWER = 0.80;

    // Servo positions
    private static final double SERVO_LOAD_POS = 0.00;
    private static final double SERVO_FEED_POS = 0.75;

    // Short feed schedule to fit ~2s “waits” (3 pulses inside ~2.0s window)
    private static final double[] FEED_START_S_SHORT = {0.20, 0.90, 1.60};
    private static final double   FEED_HOLD_S        = 0.15;
    private static final double   END_PADDING_S      = 0.20;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(-60, 34, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        DcMotor intake    = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);
        DcMotorEx shooter = (DcMotorEx) hardwareMap.get(DcMotor.class, LAUNCH_MOTOR);
        Servo feed        = hardwareMap.get(Servo.class, FEED_SERVO);
        // feed.setDirection(Servo.Direction.REVERSE); // uncomment if linkage is inverted

        // Defaults/safety
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setPower(0.0);
        shooter.setPower(0.0);
        feed.setPosition(SERVO_LOAD_POS);

        waitForStart();
        if (isStopRequested()) return;

        Action routine = drive.actionBuilder(startPose)
                // ===== Cycle 1 (to center and shoot) =====
                .splineTo(new Vector2d(0, 0), Math.toRadians(135))
                .stopAndAdd(new ShooterAndFeederAction(
                        shooter, feed, SHOOTER_POWER,
                        FEED_START_S_SHORT, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))

                // ===== Left stack (−11, 33), Intake, then return & shoot =====
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-11, 33, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(45)
                .lineToY(33)
                .splineTo(new Vector2d(0, 0), Math.toRadians(-40))
                .stopAndAdd(new ShooterAndFeederAction(
                        shooter, feed, SHOOTER_POWER,
                        FEED_START_S_SHORT, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))

                // ===== Middle stack (+11, 33), up to y=45, back to y=33, then return & shoot =====
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(11, 33, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(45)
                .lineToY(33)
                .splineTo(new Vector2d(0, 0), Math.toRadians(-40))
                .stopAndAdd(new ShooterAndFeederAction(
                        shooter, feed, SHOOTER_POWER,
                        FEED_START_S_SHORT, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))

                // ===== Right stack (+34, 33), up to y=45, back to y=33, then return & shoot =====
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(34, 33, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(45)
                .lineToY(33)
                .splineTo(new Vector2d(0, 0), Math.toRadians(-40))
                .stopAndAdd(new ShooterAndFeederAction(
                        shooter, feed, SHOOTER_POWER,
                        FEED_START_S_SHORT, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))

                .build();

        Actions.runBlocking(routine);

        // Safety park
        feed.setPosition(SERVO_LOAD_POS);
        shooter.setPower(0.0);
        intake.setPower(0.0);
    }
}