package org.firstinspires.ftc.teamcode.MainCode;

import static org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.setMotorVel;
import static org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.setMotorPower;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.ShooterAndFeederActionVel;
import org.firstinspires.ftc.teamcode.MainCode.util.TinyCsvLogger;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="BigTriRed", group="Auto")
public class BigTriRed extends LinearOpMode {

    // RC config names
    private static final String FEED_SERVO   = "feedServo";
    private static final String INTAKE_MOTOR = "IntakeMotor";
    private static final String LAUNCH_MOTOR = "LaunchMotor";

    // Tunables
    private static final double INTAKE_POWER  = 0.0;
    private static final double SHOOTER_Vel = 1380;
    private static final double SHOOTER_Vel2 = 1380;

    // Servo positions (use what worked in your tests)
    private static final double SERVO_LOAD_POS = 0.00;
    private static final double SERVO_FEED_POS = 0.12;

    // Feed schedule at the stop (seconds from start of the shooter action)
    private static final double[] FEED_START_S = {2};//2.5
    private static final double[] FEED_CON_S = {1.5, 3.5};
    private static final double   FEED_HOLD_S  = 0.7;
    private static final double   END_PADDING_S = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-48, 48, Math.toRadians(135));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);


        DcMotorEx intake  = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR);
        DcMotorEx shooter = (DcMotorEx) hardwareMap.get(DcMotor.class, LAUNCH_MOTOR);
        Servo feed        = hardwareMap.get(Servo.class, FEED_SERVO);
        // feed.setDirection(Servo.Direction.REVERSE); // if linkage inverted

        // ADDED: separate handle for intake (for logging only; same name)
        DcMotorEx intakeExForLog = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR);

        // ADDED: CSV logger
        TinyCsvLogger logger = TinyCsvLogger.create(hardwareMap, "auto_bigtri_red");

        // Safe defaults
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setPower(0.0);
        shooter.setPower(0.0);
        PIDFCoefficients pidf_cur =new PIDFCoefficients(500, 3, 0, 4);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf_cur);
        feed.setPosition(SERVO_LOAD_POS);

        waitForStart();
        if (isStopRequested()) {
            try { logger.close(); } catch (Exception ignored) {}
            return;
        }
        telemetry.addData( "TTPS", shooter.getVelocity());


        Action all = drive.actionBuilder(startPose)
                // Intake on (non-blocking; base keeps moving)

                // --- Your original path, Red side ---
                .setTangent(Math.toRadians(135))
                .stopAndAdd(setMotorVel(shooter, SHOOTER_Vel))

                .strafeTo(new Vector2d(-20, 20))

                // Shooter runs
                .stopAndAdd(new ShooterAndFeederActionVel(
                        shooter, feed,
                        SHOOTER_Vel,
                        FEED_START_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))
                .stopAndAdd(setMotorPower(intake, 0.73))
                .stopAndAdd(new ShooterAndFeederActionVel(
                        shooter, feed,
                        SHOOTER_Vel2,
                        FEED_CON_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))

                .splineToLinearHeading(new Pose2d(-6, 24,Math.toRadians(90)),Math.toRadians(90))

                .lineToY(48)
                .lineToY(45)
                .stopAndAdd(setMotorPower(intake, 0.0))
                .stopAndAdd(setMotorVel(shooter, SHOOTER_Vel))
                .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(135))

                // Shooter runs
                .stopAndAdd(new ShooterAndFeederActionVel(
                        shooter, feed,
                        SHOOTER_Vel,
                        FEED_START_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))
                .stopAndAdd(setMotorPower(intake, 0.73))
                .stopAndAdd(new ShooterAndFeederActionVel(
                        shooter, feed,
                        SHOOTER_Vel2,
                        FEED_CON_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))

                .splineToLinearHeading(new Pose2d(18, 24,Math.toRadians(90)),Math.toRadians(90))
                .lineToY(52)
                .lineToY(45)

                .stopAndAdd(setMotorPower(intake, 0.0))
                .stopAndAdd(setMotorVel(shooter, SHOOTER_Vel))
                /*
                .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(135))

                // Shooter runs
                .stopAndAdd(new ShooterAndFeederActionVel(
                        shooter, feed,
                        SHOOTER_Vel,
                        FEED_START_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))
                .stopAndAdd(setMotorPower(intake, 0.73))
                .stopAndAdd(new ShooterAndFeederActionVel(
                        shooter, feed,
                        SHOOTER_Vel,
                        FEED_CON_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))

                .splineToLinearHeading(new Pose2d(38, 24,Math.toRadians(90)),Math.toRadians(90))


                .lineToY(50)
                .lineToY(45)
                .lineToY(50)
                .stopAndAdd(setMotorPower(intake, 0.0))


                .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(135))

                //shooter runs
                .stopAndAdd(new ShooterAndFeederActionVel(
                        shooter, feed,
                        SHOOTER_Vel,
                        FEED_START_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))
                .stopAndAdd(setMotorPower(intake, 0.73))
                .stopAndAdd(new ShooterAndFeederActionVel(
                        shooter, feed,
                        SHOOTER_Vel,
                        FEED_CON_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))

                .strafeTo(new Vector2d(-48, 16))

                 */

                .build();

        // ADDED: wrap the action with per-tick logging
        Action logged = new Action() {
            @Override
            public boolean run(TelemetryPacket packet) {
                // advance odometry
                drive.updatePoseEstimate();

                // read pose + powers
                Pose2d pose = drive.localizer.getPose();
                double launchCmd = shooter.getPower(); // treat last-set power as "command" in Auto
                double intakeCmd = intake.getPower();

                // write CSV row
                logger.record(
                        "run",
                        launchCmd,
                        shooter,
                        intakeCmd,
                        intakeExForLog,
                        feed,
                        pose
                );

                // continue original chain
                return all.run(packet);
            }


        };
        telemetry.update();

        Actions.runBlocking(logged);

        // Safety park
        feed.setPosition(SERVO_LOAD_POS);
        shooter.setPower(0.0);
        intake.setPower(0.0);

        // ADDED: close the logger
        logger.close();
    }
}
/* .splineToLinearHeading(new Pose2d(-6, 24,Math.toRadians(90)),Math.toRadians(90))  .splineToLinearHeading(new Pose2d(15, 24,Math.toRadians(90)),Math.toRadians(90))        .splineToLinearHeading(new Pose2d(38, 24,Math.toRadians(90)),Math.toRadians(90))
    .splineToLinearHeading(new Pose2d(-20, 20,Math.toRadians(135)),Math.toRadians(135))

 */