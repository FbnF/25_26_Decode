package org.firstinspires.ftc.teamcode.MainCode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.ShooterAndFeederAction;
import static org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.setMotorPower;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Autonomous(name="SmallTriRed", group="Auto")
public class SmallTriRed extends LinearOpMode {

    // RC config names
    private static final String FEED_SERVO   = "feedServo";
    private static final String INTAKE_MOTOR = "IntakeMotor";
    private static final String LAUNCH_MOTOR = "LaunchMotor";

    private static final String VOLTAGE_SENSOR = "VoltageSensor";

    // Tunables
    private static final double INTAKE_POWER  = 0.6;
    private static double SHOOTER_POWER = 0.74;

    // Servo positions (use what worked in your tests)
    private static final double SERVO_LOAD_POS = 0.00;
    private static final double SERVO_FEED_POS = 0.75;

    // Feed schedule at the stop (seconds from start of the shooter action)
    private static final double[] FEED_START_S = {4.52, 6.52, 9.52};
    private static final double   FEED_HOLD_S  = 0.7;
    private static final double   END_PADDING_S = 1.0;

    private static final  double FULL_POWER = 12;

    private double CURRENT_POWER;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(60, 12, Math.toRadians(145));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        DcMotor intake        = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);
        DcMotorEx shooter     = hardwareMap.get(DcMotorEx.class, LAUNCH_MOTOR);
        Servo feed            = hardwareMap.get(Servo.class, FEED_SERVO);
       // VoltageSensor voltageSensor = hardwareMap.get(VoltageSensor.class, VOLTAGE_SENSOR);
        // feed.setDirection(Servo.Direction.REVERSE); // if linkage inverted
        //CURRENT_POWER = voltageSensor.getVoltage();
        //SHOOTER_POWER = SHOOTER_POWER + ((FULL_POWER-CURRENT_POWER)*0.05);
        // Safe defaults
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0.0);
        shooter.setPower(0.0);
        feed.setPosition(SERVO_LOAD_POS);
        telemetry.addData("SHOOTER_POWER", SHOOTER_POWER);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        Action all = drive.actionBuilder(startPose)
                // Intake on (non-blocking; base keeps moving)
                //.stopAndAdd(setMotorPower(intake, INTAKE_POWER))

                // --- Your original path, Red side ---
                .setTangent(0)

//                .strafeTo(new Vector2d(-50, 41))
//                .waitSeconds(5)
//                .strafeTo(new Vector2d(-50, 46))
//                .waitSeconds(5)
//                .strafeTo(new Vector2d(-45, 46))
//                .waitSeconds(20)


                // Pause base: shooter + 3 servo pulses (then shooter stops)
                .stopAndAdd(new ShooterAndFeederAction(
                        shooter, feed,
                        SHOOTER_POWER,
                        FEED_START_S, FEED_HOLD_S, END_PADDING_S,
                        SERVO_LOAD_POS, SERVO_FEED_POS))
              //  .setTangent(0)
               // .splineTo(new Vector2d(48, 24), Math.PI*3 / 2)
                    .strafeTo(new Vector2d(38, 20))

       //         .strafeTo(new Vector2d(12, 24))

                .build();

        Actions.runBlocking(all);

        // Safety park
        feed.setPosition(SERVO_LOAD_POS);
        shooter.setPower(0.0);
        intake.setPower(0.0);
    }
}