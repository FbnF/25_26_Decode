package org.firstinspires.ftc.teamcode.MainCode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.ShooterAndFeederAction;

@Disabled
@Autonomous(name="SmallTriRed", group="Auto")
public class SmallTriRed extends LinearOpMode {

    // RC config names
    private static final String FEED_SERVO   = "feedServo";
    private static final String INTAKE_MOTOR = "IntakeMotor";
    private static final String LAUNCH_MOTOR = "LaunchMotor";

    // Alliance goal tag (20 = blue, 24 = red)
    private static final int GOAL_TAG_ID = 24;

    // Tunables
    private static final double INTAKE_POWER  = 0.6;

    // Servo positions (use what worked in your tests)
    private static final double SERVO_LOAD_POS = 0.00;
    private static final double SERVO_FEED_POS = 0.75;

    // Shooting
    private static final int NUM_SHOTS = 3;
    private static final double FEED_HOLD_S  = 0.7;
    private static final double END_PADDING_S = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(60, 12, Math.toRadians(145));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        DcMotor intake    = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, LAUNCH_MOTOR);
        Servo feed        = hardwareMap.get(Servo.class, FEED_SERVO);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        // Safe defaults
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setPower(0.0);
        shooter.setPower(0.0);
        feed.setPosition(SERVO_LOAD_POS);

        waitForStart();
        if (isStopRequested()) return;

        Action all = drive.actionBuilder(startPose)

                .setTangent(0)

                // Shooter + feed based on Limelight distance, fires when at speed
                .stopAndAdd(new ShooterAndFeederAction(
                        shooter, feed, limelight,
                        GOAL_TAG_ID,
                        NUM_SHOTS,
                        SERVO_LOAD_POS, SERVO_FEED_POS,
                        FEED_HOLD_S, END_PADDING_S
                ))

                .strafeTo(new Vector2d(38, 20))

                .build();

        Actions.runBlocking(all);

        // Safety park
        feed.setPosition(SERVO_LOAD_POS);
        shooter.setPower(0.0);
        intake.setPower(0.0);
    }
}