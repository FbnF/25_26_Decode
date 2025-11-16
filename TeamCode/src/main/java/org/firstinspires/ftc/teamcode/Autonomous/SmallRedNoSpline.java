package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.setMotorPower;

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

import org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.ShooterAndFeederAction;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="SmallTriRedNoSpline", group="Auto")
public class SmallRedNoSpline extends LinearOpMode {

    // RC config names
    private static final String FEED_SERVO   = "feedServo";
    private static final String INTAKE_MOTOR = "IntakeMotor";
    private static final String LAUNCH_MOTOR = "LaunchMotor";

    //private static final String VOLTAGE_SENSOR = "VoltageSensor";

    // Tunables
    private static final double INTAKE_POWER  = 0.6;
    private static double SHOOTER_POWER = 0.75;

    // Servo positions (use what worked in your tests)
    private static final double SERVO_LOAD_POS = 0.0;
    private static final double SERVO_FEED_POS = 0.75;

    // Feed schedule at the stop (seconds from start of the shooter action)
    private static final double[] FEED_START_S = {4.52, 6.52, 9.52, 12.52};
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
        intake.setPower(0.0);
        shooter.setPower(0.0);
        feed.setPosition(SERVO_LOAD_POS);

        // proportional compensation: keep motor voltage constant
       /* double vbat = (battery != null) ? battery.getVoltage() : 12.0;
        if (!Double.isFinite(vbat) || vbat <= 0) vbat = 12.0;
        SHOOTER_POWER = Math.min(1.0, SHOOTER_POWER * (12.0 / vbat));*/

        telemetry.addData("SHOOTER_POWER", SHOOTER_POWER);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        Action all = drive.actionBuilder(startPose)
                // Intake on (non-blocking; base keeps moving)
                //.stopAndAdd(setMotorPower(intake, INTAKE_POWER))
                .stopAndAdd(setMotorPower(intake, 1.0))
                .stopAndAdd(setMotorPower(shooter, 0.74))
                .turn(Math.toRadians(-28))
               .stopAndAdd(new ShooterAndFeederAction(
                        shooter, feed,
                        SHOOTER_POWER,
                        FEED_START_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS,
                        MAX_VOLTAGE))
                .turn(Math.toRadians(28))

                //      .turn(Math.toRadians(-45))
                //first artifact round
                //Pick up artifacts.
                .splineTo(new Vector2d(33, 36), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(60, 12))
                .turn(Math.toRadians(-28))
                .stopAndAdd(new ShooterAndFeederAction(
                        shooter, feed,
                        SHOOTER_POWER,
                        FEED_START_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS,
                        MAX_VOLTAGE))
                .turn(Math.toRadians(28))
                /*.stopAndAdd(setMotorPower(intake, 1.0))
                .turn(Math.toRadians(-90))
                .strafeTo(new Vector2d(33, 52))
                //End pick up.
                .strafeTo(new Vector2d(60,16))
                .turn(Math.toRadians(-35))*/
             /*   .stopAndAdd(new ShooterAndFeederAction(
                        shooter, feed,
                        SHOOTER_POWER,
                        FEED_START_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS, MAX_VOLTAGE))

                //new pickup
                .setTangent(90)
                .turn(Math.toRadians(-35))
                .setTangent(90)
                .strafeTo(new Vector2d(15,0))
                //.lineToX(15)
                // .turn(Math.toRadians(-55))
                .strafeTo(new Vector2d(15, 52))*/
                .build();

        Actions.runBlocking(all);

        // Safety park
        feed.setPosition(SERVO_LOAD_POS);
        shooter.setPower(0.0);
        intake.setPower(0.0);
    }
}