package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.setMotorPower;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.ShooterAndFeederAction;
import org.firstinspires.ftc.teamcode.MainCode.util.TinyCsvLogger;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="BigTriRedComplex", group="Auto")
public class BigTriRedComplex extends LinearOpMode {

    // RC config names
    private static final String FEED_SERVO   = "feedServo";
    private static final String INTAKE_MOTOR = "IntakeMotor";
    private static final String LAUNCH_MOTOR = "LaunchMotor";

    // Alliance goal tag (20 = blue, 24 = red)
    private static final int GOAL_TAG_ID = 24;

    // Servo positions (use what worked in your tests)
    private static final double SERVO_LOAD_POS = 0.00;
    private static final double SERVO_FEED_POS = 0.12;

    private static final double FEED_HOLD_S  = 0.7;
    private static final double END_PADDING_S = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-48, 48, Math.toRadians(135));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        DcMotorEx intake  = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR);
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, LAUNCH_MOTOR);
        Servo feed        = hardwareMap.get(Servo.class, FEED_SERVO);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        TinyCsvLogger logger = TinyCsvLogger.create(hardwareMap, "auto_bigtri_red_complex");

        // Safe defaults
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setPower(0.0);
        shooter.setPower(0.0);
        feed.setPosition(SERVO_LOAD_POS);

        waitForStart();
        if (isStopRequested()) {
            try { logger.close(); } catch (Exception ignored) {}
            return;
        }

        Action all = drive.actionBuilder(startPose)

                // --- Your original path, Red side ---
                .setTangent(Math.toRadians(135))

                .strafeTo(new Vector2d(-20, 20))

                // Shoot 1 (Limelight sets TPS; feed when at speed)
                .stopAndAdd(new ShooterAndFeederAction(
                        shooter, feed, limelight,
                        GOAL_TAG_ID,
                        1,
                        SERVO_LOAD_POS, SERVO_FEED_POS,
                        FEED_HOLD_S, END_PADDING_S
                ))

                .stopAndAdd(setMotorPower(intake, 0.73))

                // Shoot 2
                .stopAndAdd(new ShooterAndFeederAction(
                        shooter, feed, limelight,
                        GOAL_TAG_ID,
                        1,
                        SERVO_LOAD_POS, SERVO_FEED_POS,
                        FEED_HOLD_S, END_PADDING_S
                ))

                .splineToLinearHeading(new Pose2d(-6, 24, Math.toRadians(90)), Math.toRadians(90))

                .lineToY(48)
                .lineToY(45)

                .stopAndAdd(setMotorPower(intake, 0.0))

                .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(135))

                // Shoot 3
                .stopAndAdd(new ShooterAndFeederAction(
                        shooter, feed, limelight,
                        GOAL_TAG_ID,
                        1,
                        SERVO_LOAD_POS, SERVO_FEED_POS,
                        FEED_HOLD_S, END_PADDING_S
                ))

                .stopAndAdd(setMotorPower(intake, 0.73))

                // Shoot 4
                .stopAndAdd(new ShooterAndFeederAction(
                        shooter, feed, limelight,
                        GOAL_TAG_ID,
                        1,
                        SERVO_LOAD_POS, SERVO_FEED_POS,
                        FEED_HOLD_S, END_PADDING_S
                ))

                .splineToLinearHeading(new Pose2d(18, 24, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(52)
                .lineToY(45)

                .stopAndAdd(setMotorPower(intake, 0.0))

                .build();

        Action logged = new Action() {
            @Override
            public boolean run(TelemetryPacket packet) {
                Pose2d pose = drive.localizer.getPose();

                logger.record(
                        "run",
                        shooter.getVelocity(),
                        shooter,
                        intake.getPower(),
                        intake,
                        feed,
                        pose
                );

                return all.run(packet);
            }
        };

        Actions.runBlocking(logged);

        // Safety park
        feed.setPosition(SERVO_LOAD_POS);
        shooter.setPower(0.0);
        intake.setPower(0.0);

        logger.close();
    }
}