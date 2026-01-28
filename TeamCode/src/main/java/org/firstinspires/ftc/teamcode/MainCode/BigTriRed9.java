package org.firstinspires.ftc.teamcode.MainCode;

import static org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.setCRServoPower;
import static org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.setMotorPower;
import static org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.setMotorVel;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.ShooterAndFeederCombined;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous(name = "BigTriRED9", group = "BigTriRed")
public class BigTriRed9 extends LinearOpMode {

    private static final String FEED_SERVO   = "feedServo";
    private static final String INTAKE_MOTOR = "IntakeMotor";
    private static final String LAUNCH_MOTOR = "LaunchMotor";
    private static final String SIDE_SERVO = "sideServo";
    private static final String DISTANCE_SENSOR = "RangeSensor";

    // Tunables
    private static final double INTAKE_POWER  = 0.0;
    private static final double SHOOTER_Vel = 1325;

    // Feed schedule at the stop (seconds from start of the shooter action)
    private static final double WAIT_TIME = 0.8;
    private static final double WAIT_TIME_Start = 1.1;
    private static final double SHOOT_TIME = 4.2;
    private static final double sidePower = -0.9;



    @Override
    public void runOpMode() {
        // Start at origin, heading = 0 rad (east)
        Pose2d startPose = new Pose2d(-57, 36, Math.toRadians(90));//Pose2d startPose = new Pose2d(-48, 48, Math.toRadians(135));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        DcMotor intake        = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);
        DcMotorEx shooter     = (DcMotorEx) hardwareMap.get(DcMotor.class, LAUNCH_MOTOR);
        CRServo feed            = hardwareMap.get(CRServo.class, FEED_SERVO);
        CRServo side            = hardwareMap.get(CRServo.class, SIDE_SERVO);
        DistanceSensor distance = hardwareMap.get(DistanceSensor.class, DISTANCE_SENSOR);

        // Build one continuous action so pose/tangent carry correctly between segments.
        Action all = drive.actionBuilder(startPose)
                // First strafe and shoot
                .setTangent(Math.toRadians(135))
                .stopAndAdd(setMotorVel(shooter, SHOOTER_Vel)) //start up motor
                .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(135))

                // Shooter runs
                .stopAndAdd(new  ShooterAndFeederCombined(
                        shooter, intake,
                        feed, side, distance,
                        SHOOTER_Vel, sidePower,
                        WAIT_TIME_Start,SHOOT_TIME))

                .stopAndAdd(setMotorPower(intake, 1.0))
                .stopAndAdd(setCRServoPower(side,1.0))
                // collect first spike line
                .splineToLinearHeading(new Pose2d(-5, 24,Math.toRadians(90)),Math.toRadians(90))
                .stopAndAdd(setMotorVel(shooter, SHOOTER_Vel))
                .lineToY(48)
                .lineToY(45)
                .stopAndAdd(setMotorPower(intake, 0.0))
                .stopAndAdd(setCRServoPower(side,0.0))
                .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(135))

                // Shooter runs
                .stopAndAdd(new  ShooterAndFeederCombined(
                        shooter, intake,
                        feed, side, distance,
                        SHOOTER_Vel, sidePower,
                        WAIT_TIME,SHOOT_TIME))
                .stopAndAdd(setMotorPower(intake, 1.0))
                .stopAndAdd(setCRServoPower(side,1.0))

                // collect second spike line

                .splineToLinearHeading(new Pose2d(17.5, 24,Math.toRadians(90)),Math.toRadians(90))
                .stopAndAdd(setMotorVel(shooter, SHOOTER_Vel))
                .lineToY(52)
                .lineToY(45)
                .stopAndAdd(setMotorPower(intake, 0.0))
                .stopAndAdd(setCRServoPower(side,0.0))

                .strafeToLinearHeading(new Vector2d(-29, 11.5), Math.toRadians(130))
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