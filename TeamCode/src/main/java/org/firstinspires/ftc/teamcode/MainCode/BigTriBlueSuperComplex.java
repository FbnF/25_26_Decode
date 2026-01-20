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
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.ShooterAndCRFeederActionVel;
import org.firstinspires.ftc.teamcode.MainCode.util.TinyCsvLogger;
import org.firstinspires.ftc.teamcode.MecanumDrive;



@Autonomous(name = "BigTriBlueSuperComplex", group = "Auto")
public class BigTriBlueSuperComplex extends LinearOpMode {

    private static final String FEED_SERVO   = "feedServo";
    private static final String INTAKE_MOTOR = "IntakeMotor";
    private static final String LAUNCH_MOTOR = "LaunchMotor";

    // Tunables
    private static final double INTAKE_POWER  = 0.0;
    private static final double SHOOTER_Vel = 1340;
    // Servo positions (use what worked in your tests)
    private static final double SERVO_LOAD_POS = 0.0;
    private static final double SERVO_FEED_POS = 0.12;

    // Feed schedule at the stop (seconds from start of the shooter action)
    private static final double[] FEED_START_S = {0.5, 1};
    private static final double[] FEED_CON_S = {1};
    private static final double   FEED_HOLD_S  = 0.7;
    private static final double   END_PADDING_S = 0.4;

    @Override
    public void runOpMode() {
        // Start at origin, heading = 0 rad (east)
        Pose2d startPose = new Pose2d(-48, -48, Math.toRadians(225));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        DcMotor intake        = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);
        DcMotorEx shooter     = (DcMotorEx) hardwareMap.get(DcMotor.class, LAUNCH_MOTOR);
        CRServo feed            = hardwareMap.get(CRServo.class, FEED_SERVO);

        // Build one continuous action so pose/tangent carry correctly between segments.
        Action all = drive.actionBuilder(startPose)
                // First strafe and shoot
                .setTangent(Math.toRadians(225))
                .stopAndAdd(setMotorVel(shooter, SHOOTER_Vel)) //start up motor
                .strafeTo(new Vector2d(-20, -20))

                // Shooter runs
                .stopAndAdd(new ShooterAndCRFeederActionVel(
                        shooter, feed,
                        SHOOTER_Vel,
                        FEED_START_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))
                .stopAndAdd(setMotorPower(intake, 0.73))// get last ball out of the intake
                .stopAndAdd(new ShooterAndCRFeederActionVel(
                        shooter, feed,
                        SHOOTER_Vel,
                       FEED_CON_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))

                // collect first spike line
                .splineToLinearHeading(new Pose2d(-10, -24,Math.toRadians(270)),Math.toRadians(270))
                .lineToY(-48)
                .lineToY(-45)
                .stopAndAdd(setMotorPower(intake, 0.0))
                .stopAndAdd(setMotorVel(shooter, SHOOTER_Vel))
                .strafeToLinearHeading(new Vector2d(-20, -20), Math.toRadians(225))

                // Shooter runs
                .stopAndAdd(new ShooterAndCRFeederActionVel(
                        shooter, feed,
                        SHOOTER_Vel,
                        FEED_START_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))
                .stopAndAdd(setMotorPower(intake, 0.73))// get last ball out of the intake
                .stopAndAdd(new ShooterAndCRFeederActionVel(
                        shooter, feed,
                        SHOOTER_Vel,
                        FEED_CON_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))
                //Clear
                .strafeToLinearHeading(new Vector2d(0, -40), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(0, -54), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(0, -20), Math.toRadians(180))

                // collect second spike line

                .splineToLinearHeading(new Pose2d(13.5, -24,Math.toRadians(270)),Math.toRadians(270))
                .lineToY(-52)
                .lineToY(-45)
                .stopAndAdd(setMotorPower(intake, 0.0))
                .stopAndAdd(setMotorVel(shooter, SHOOTER_Vel))
                .strafeToLinearHeading(new Vector2d(-20,-20), Math.toRadians(225))
                // Shooter runs
                .stopAndAdd(new ShooterAndCRFeederActionVel(
                        shooter, feed,
                        SHOOTER_Vel,
                        FEED_START_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))
                .stopAndAdd(setMotorPower(intake, 0.73))// get last ball out of the intake
                .stopAndAdd(new ShooterAndCRFeederActionVel(
                        shooter, feed,
                        SHOOTER_Vel,
                        FEED_CON_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))

                //Collect
                .splineToLinearHeading(new Pose2d(20, -50,Math.toRadians(270)), Math.toRadians(270))
                .stopAndAdd(setMotorPower(intake, 0.0))
                .stopAndAdd(setMotorVel(shooter, SHOOTER_Vel))
                .strafeToLinearHeading(new Vector2d(20, -60), Math.toRadians(270))

                // Shooter runs
                .stopAndAdd(new ShooterAndCRFeederActionVel(
                        shooter, feed,
                        SHOOTER_Vel,
                        FEED_START_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))
                .stopAndAdd(setMotorPower(intake, 0.73))// get last ball out of the intake
                .stopAndAdd(new ShooterAndCRFeederActionVel(
                        shooter, feed,
                        SHOOTER_Vel,
                        FEED_CON_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))
                //Clear
                .strafeToLinearHeading(new Vector2d(-20,-20), Math.toRadians(225))
                .strafeToLinearHeading(new Vector2d(0, -40), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(0, -54), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(0, -40), Math.toRadians(180))
                //Collect
                .splineToLinearHeading(new Pose2d(20, -50,Math.toRadians(270)), Math.toRadians(270))
                .stopAndAdd(setMotorPower(intake, 0.0))
                .stopAndAdd(setMotorVel(shooter, SHOOTER_Vel))
                .strafeToLinearHeading(new Vector2d(20, -60), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-36, -12), Math.toRadians(250))
                // Shooter runs
                .stopAndAdd(new ShooterAndCRFeederActionVel(
                        shooter, feed,
                        SHOOTER_Vel,
                        FEED_START_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))
                .stopAndAdd(setMotorPower(intake, 0.73))// get last ball out of the intake
                .stopAndAdd(new ShooterAndCRFeederActionVel(
                        shooter, feed,
                        SHOOTER_Vel,
                        FEED_CON_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))

                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(all);
    }
}