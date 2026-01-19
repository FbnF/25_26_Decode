package org.firstinspires.ftc.teamcode.MainCode;

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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MainCode.config.TagConfig;
import org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl;
import org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.ShooterAndFeederAction;
import org.firstinspires.ftc.teamcode.MainCode.vision.AprilTagService;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="SmallRedSimple", group="Auto")
public class SmallRedSimple extends LinearOpMode {

    // RC config names
    private static final String FEED_SERVO   = "feedServo";
    private static final String INTAKE_MOTOR = "IntakeMotor";
    private static final String LAUNCH_MOTOR = "LaunchMotor";

    //private static final String VOLTAGE_SENSOR = "VoltageSensor";

    // Tunables
    private static final double INTAKE_POWER  = 0.73;

    private static double SHOOTER_POWER = 0.78;
    private static double SHOOTER_VEL = 1740;

    private static double SHOOTER_VEL_R2 = 1770;


    private static double SHOOTER_VEL_SEC = 1755.0;


    // Servo positions (use what worked in your tests)
    private static final double SERVO_LOAD_POS = 0.0;
    private static final double SERVO_FEED_POS = 0.12;

    private static final double ANGLE_OF_TURN = 27.5;

    // Feed schedule at the stop (seconds from start of the shooter action)
    private static final double[] FEED_START_S = {2, 4};
    private static final double[] FEED_START_S_FIRST = {1.5};
    private static final double[] FEED_START_S_FIRST_2 = {2};
    private static final double   FEED_HOLD_S  = 0.7;
    private static final double   END_PADDING_S = 1.0;

    private static final double MAX_VOLTAGE = 12.5;

    double CURRENT_VOLTAGE = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(60, 16, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        DcMotor intake        = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);
        DcMotorEx shooter     = (DcMotorEx) hardwareMap.get(DcMotor.class, LAUNCH_MOTOR);
        Servo feed            = hardwareMap.get(Servo.class, FEED_SERVO);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        // VoltageSensor battery = hardwareMap.get(VoltageSensor.class, VOLTAGE_SENSOR); // read battery

        // Safe defaults
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidf_cur = new PIDFCoefficients(500, 3, 0, 4);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf_cur);
        intake.setPower(0.0);
        shooter.setPower(0.0);
        feed.setPosition(SERVO_LOAD_POS);


        // proportional compensation: keep motor voltage constant
       /* double vbat = (battery != null) ? battery.getVoltage() : 12.0;
        if (!Double.isFinite(vbat) || vbat <= 0) vbat = 12.0;
        SHOOTER_POWER = Math.min(1.0, SHOOTER_POWER * (12.0 / vbat));*/

        telemetry.addData("SHOOTER_POWER", SHOOTER_POWER);



        waitForStart();
        telemetry.addData("SHOOTER_VELOCITY", shooter.getVelocity());
        telemetry.update();
        if (isStopRequested()) return;

        Action all = drive.actionBuilder(startPose)
                .stopAndAdd(setMotorPower(intake, 0.0))
                .stopAndAdd(setMotorVel(shooter, SHOOTER_VEL))
                .strafeToLinearHeading(new Vector2d(51, 10), Math.toRadians(154.5) )
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
                .setTangent(Math.toRadians(154.5))
                .strafeToLinearHeading(new Vector2d(49, 25), Math.toRadians(155.5))
                .build();

        Actions.runBlocking(all);

        // Safety park
        feed.setPosition(SERVO_LOAD_POS);
        shooter.setPower(0.0);
        intake.setPower(0.0);
    }
}