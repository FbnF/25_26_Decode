package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.setMotorPower;
import static org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.setMotorVel;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="SmallTriRedNoSpline", group="Auto")
public class SmallRedNoSpline extends LinearOpMode {

    // RC config names
    private static final String FEED_SERVO   = "feedServo";
    private static final String INTAKE_MOTOR = "IntakeMotor";
    private static final String LAUNCH_MOTOR = "LaunchMotor";

    // Tunables
    private static final double INTAKE_POWER  = 0.73;

    private static double SHOOTER_POWER = 0.78; // kept (not used in velocity path)
    private static double SHOOTER_VEL = 1910;
    private static double SHOOTER_VEL_SEC = 1850;

    // Servo positions
    private static final double SERVO_LOAD_POS = 0.0;
    private static final double SERVO_FEED_POS = 0.12;

    private static final double ANGLE_OF_TURN = 27.5;

    // Feed schedule at the stop (seconds from start of the shooter action)
    private static final double[] FEED_START_S = {2.5, 4.5};
    private static final double[] FEED_START_S_FIRST = {2.7};
    private static final double   FEED_HOLD_S  = 0.7;
    private static final double   END_PADDING_S = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(60, 16, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        DcMotor intake        = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);
        DcMotorEx shooter     = hardwareMap.get(DcMotorEx.class, LAUNCH_MOTOR);
        Servo feed            = hardwareMap.get(Servo.class, FEED_SERVO);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        // Safe defaults
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setPower(0.0);
        shooter.setPower(0.0);
        feed.setPosition(SERVO_LOAD_POS);

        telemetry.addData("SHOOTER_POWER", SHOOTER_POWER);

        waitForStart();

        telemetry.addData("SHOOTER_VELOCITY", shooter.getVelocity());
        telemetry.update();

        if (isStopRequested()) return;

        Action all = drive.actionBuilder(startPose)
                .stopAndAdd(setMotorPower(intake, 0.0))
                .stopAndAdd(setMotorVel(shooter, SHOOTER_VEL))
                .turn(Math.toRadians(ANGLE_OF_TURN * -1))
                .stopAndAdd(new AutoMotorControl.ShooterAndFeederActionVel(
                        shooter, feed,
                        SHOOTER_VEL,
                        FEED_START_S_FIRST, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))
                .stopAndAdd(setMotorPower(intake, INTAKE_POWER))
                .stopAndAdd(new AutoMotorControl.ShooterAndFeederActionVel(
                        shooter, feed,
                        SHOOTER_VEL_SEC,
                        FEED_START_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))
                .turn(Math.toRadians(ANGLE_OF_TURN))
                .setTangent(Math.toRadians(180))
                .stopAndAdd(setMotorPower(intake, INTAKE_POWER))
                .splineTo(new Vector2d(34, 36), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .lineToY(52)
                .stopAndAdd(setMotorPower(intake, 0.0))
                .turn(Math.toRadians(90 - ANGLE_OF_TURN))
                .strafeTo(new Vector2d(58, 16))
                .stopAndAdd(setMotorVel(shooter, SHOOTER_VEL))
                .stopAndAdd(new AutoMotorControl.ShooterAndFeederActionVel(
                        shooter, feed,
                        SHOOTER_VEL,
                        FEED_START_S_FIRST, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))
                .stopAndAdd(setMotorPower(intake, INTAKE_POWER))
                .stopAndAdd(new AutoMotorControl.ShooterAndFeederActionVel(
                        shooter, feed,
                        SHOOTER_VEL_SEC,
                        FEED_START_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))
                .setTangent(Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(38, 25), Math.toRadians(90))
                .build();

        Actions.runBlocking(all);

        // Safety park
        feed.setPosition(SERVO_LOAD_POS);
        shooter.setPower(0.0);
        intake.setPower(0.0);
    }
}