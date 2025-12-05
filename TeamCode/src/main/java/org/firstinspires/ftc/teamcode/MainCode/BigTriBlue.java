package org.firstinspires.ftc.teamcode.MainCode;

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
import org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.ShooterAndFeederAction;
import org.firstinspires.ftc.teamcode.MainCode.util.TinyCsvLogger;

import static org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.setMotorPower;
@Disabled
@Autonomous(name="BigTriBlue", group="Auto")

public class BigTriBlue extends LinearOpMode {

    // RC config names
    private static final String FEED_SERVO   = "feedServo";
    private static final String INTAKE_MOTOR = "IntakeMotor";
    private static final String LAUNCH_MOTOR = "LaunchMotor";

    // Tunables
    private static final double INTAKE_POWER  = 1.0;
    private static final double SHOOTER_POWER = 0.57;

    // Servo positions
    private static final double SERVO_LOAD_POS = 0.00;
    private static final double SERVO_FEED_POS = 0.75;

    // Feed schedule (seconds from start of shooter/feeder action)
    private static final double[] FEED_START_S = {3.0, 6.0, 9.0};
    private static final double   FEED_HOLD_S  = 0.7;
    private static final double   END_PADDING_S = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-48, -48, Math.toRadians(225));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        DcMotorEx intake  = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR);
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, LAUNCH_MOTOR);
        Servo feed        = hardwareMap.get(Servo.class, FEED_SERVO);
        // feed.setDirection(Servo.Direction.REVERSE); // if the linkage is inverted

        // >>> ADDED: duplicate handle for intake (for logging only)
        DcMotorEx intakeExForLog = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR);

        // >>> ADDED: create CSV logger (Auto)
        TinyCsvLogger logger = TinyCsvLogger.create(hardwareMap, "auto_bigtri_blue");

        // Defaults/safety
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0.0);
        shooter.setPower(0.0);
        feed.setPosition(SERVO_LOAD_POS);

        waitForStart();
        if (isStopRequested()) {
            try { logger.close(); } catch (Exception ignored) {}
            return;
        }

        Action routine = drive.actionBuilder(startPose)
                // Start intake; non-blocking
                //.stopAndAdd(setMotorPower(intake, INTAKE_POWER))

//                Figure out the location

                // Drive to launch position
                .lineToY(-20)

//                .waitSeconds(5)
//                .strafeTo(new Vector2d(-80, -15))
//                .waitSeconds(5)
//                .strafeTo(new Vector2d(-80, -20))
//                .waitSeconds(5)
//                .strafeTo(new Vector2d(-90, -25))

                // Pause base: shooter + 3 servo pulses
                .stopAndAdd(new ShooterAndFeederAction(
                        shooter, feed,
                        SHOOTER_POWER,
                        FEED_START_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))

//                .waitSeconds(5)
//                .strafeTo(new Vector2d(-80, 10))
//                .waitSeconds(5)
                .strafeTo(new Vector2d(-48, -16))
//                .waitSeconds(5)
//                .strafeTo(new Vector2d(-80, 15))

                .build();

        // >>> ADDED: wrap routine in per-tick logger <<<
        Action logged = new Action() {
            @Override
            public boolean run(TelemetryPacket packet) {
                drive.updatePoseEstimate();

                Pose2d pose = drive.localizer.getPose();
                double launchCmd = shooter.getPower();
                double intakeCmd = intake.getPower();

                logger.record(
                        "run",
                        launchCmd,
                        shooter,
                        intakeCmd,
                        intakeExForLog,
                        feed,
                        pose
                );

                return routine.run(packet);
            }
        };

        Actions.runBlocking(logged);

        // Safety park
        feed.setPosition(SERVO_LOAD_POS);
        shooter.setPower(0.0);
        intake.setPower(0.0);

        // >>> ADDED: close logger
        logger.close();
    }
}