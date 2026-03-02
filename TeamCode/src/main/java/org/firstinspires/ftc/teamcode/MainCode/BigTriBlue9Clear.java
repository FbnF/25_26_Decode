package org.firstinspires.ftc.teamcode.MainCode;

import static org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.setCRServoPower;
import static org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.setMotorVel;
import static org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.setMotorPower;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.ShooterAndFeederCombined;
import org.firstinspires.ftc.teamcode.MecanumDrive;



@Autonomous(name = "BigTriBLUE9Clear", group = "BigTriBlue")
public class BigTriBlue9Clear extends LinearOpMode {

    private static final String FEED_SERVO   = "feedServo";
    private static final String INTAKE_MOTOR = "IntakeMotor";
    private static final String LAUNCH_MOTOR = "LaunchMotor";
    private static final String SIDE_SERVO = "sideServo";
    private static final String DISTANCE_SENSOR = "RangeSensor";

    // Tunables
    private static final double INTAKE_POWER  = 0.0;
    private static final double SHOOTER_Vel = 1200;
    private static final double SHOOTER_Vel_2 = 1220;

    // Feed schedule at the stop (seconds from start of the shooter action)
    private static final double WAIT_TIME = 0.7;
    private static final double WAIT_TIME_Start = 0.7;
    private static final double SHOOT_TIME = 4.5;
    private static final double sidePower = -0.9;


    @Override
    public void runOpMode() {
        // Start at origin, heading = 0 rad (east)
        Pose2d startPose = new Pose2d(-57, -36, Math.toRadians(270)); //-48, -48, Math.toRadians(225));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        DcMotor intake        = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);
        DcMotorEx shooter     = (DcMotorEx) hardwareMap.get(DcMotor.class, LAUNCH_MOTOR);
        CRServo feed            = hardwareMap.get(CRServo.class, FEED_SERVO);
        CRServo side            = hardwareMap.get(CRServo.class, SIDE_SERVO);
        DistanceSensor distance = hardwareMap.get(DistanceSensor.class, DISTANCE_SENSOR);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidf_cur = new PIDFCoefficients(500, 3, 0, 4);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf_cur);
        intake.setPower(0.0);
        shooter.setPower(0.0);
        feed.setPower(0);

        // Build one continuous action so pose/tangent carry correctly between segments.
        Action all = drive.actionBuilder(startPose)
                // First strafe and shoot
                .setTangent(Math.toRadians(225))
                .stopAndAdd(setMotorVel(shooter, SHOOTER_Vel)) //start up motor
                .strafeToLinearHeading(new Vector2d(-20, -20), Math.toRadians(232))

                // Shooter runs
                .stopAndAdd(new  ShooterAndFeederCombined(
                        shooter, intake,
                        feed, side, distance,
                        SHOOTER_Vel, sidePower,
                        WAIT_TIME_Start,SHOOT_TIME))

                .stopAndAdd(setMotorPower(intake, 1.0))
                .stopAndAdd(setCRServoPower(side,1.0))
                // collect first spike line
                .splineToLinearHeading(new Pose2d(-8.5, -24,Math.toRadians(270)),Math.toRadians(263))
                .stopAndAdd(setMotorVel(shooter, SHOOTER_Vel))
                .lineToY(-48)
                .lineToY(-45)
                .stopAndAdd(setMotorPower(intake, 0.0))
                .stopAndAdd(setCRServoPower(side,0.0))
                //Clear
                .strafeToLinearHeading(new Vector2d(2, -40), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(2, -54), Math.toRadians(0))
                .waitSeconds(0.15)
                .strafeToLinearHeading(new Vector2d(-20, -20), Math.toRadians(226))

                // Shooter runs
                .stopAndAdd(new  ShooterAndFeederCombined(
                        shooter, intake,
                        feed, side, distance,
                        SHOOTER_Vel_2, sidePower,
                        WAIT_TIME,SHOOT_TIME))
                .stopAndAdd(setMotorPower(intake, 1.0))
                .stopAndAdd(setCRServoPower(side,1.0))

                // collect second spike line

                .splineToLinearHeading(new Pose2d(16, -24,Math.toRadians(266)),Math.toRadians(266))
                .stopAndAdd(setMotorVel(shooter, SHOOTER_Vel))
                .lineToY(-52)
                .lineToY(-45)
                .stopAndAdd(setMotorPower(intake, 0.0))
                .stopAndAdd(setCRServoPower(side,0.0))

                .strafeToLinearHeading(new Vector2d(-29, -11.5), Math.toRadians(235))
                // Shooter runs
                .stopAndAdd(new  ShooterAndFeederCombined(
                        shooter, intake,
                        feed, side, distance,
                        SHOOTER_Vel, sidePower,
                        WAIT_TIME,SHOOT_TIME))

                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(all);
    }
}