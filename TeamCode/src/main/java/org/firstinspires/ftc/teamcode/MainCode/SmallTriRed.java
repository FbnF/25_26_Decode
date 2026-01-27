package org.firstinspires.ftc.teamcode.MainCode;

import static org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.setMotorPower;
import static org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.setMotorVel;

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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MainCode.config.TagConfig;
import org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl;
import org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.ShooterAndFeederAction;
import org.firstinspires.ftc.teamcode.MainCode.vision.AprilTagService;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="SmallTriRed", group="Auto")
public class SmallTriRed extends LinearOpMode {

    // RC config names
    private static final String FEED_SERVO   = "feedServo";
    private static final String INTAKE_MOTOR = "IntakeMotor";
    private static final String LAUNCH_MOTOR = "LaunchMotor";
    private static final String SIDE_SERVO = "sideServo";

    //private static final String VOLTAGE_SENSOR = "VoltageSensor";

    // Tunables
    public static final double INTAKE_POWER  = 0.73;
    public static final double SIDE_POWER = -0.85;

    public static double SHOOTER_POWER = 0.78;
    public static double SHOOTER_VEL = 1760;


    public static double WaitTime = 5.5;
    public static double StartWaitTime = 2;


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(60, 16, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        DcMotor intake        = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);
        DcMotorEx shooter     = (DcMotorEx) hardwareMap.get(DcMotor.class, LAUNCH_MOTOR);
        CRServo feed            = hardwareMap.get(CRServo.class, FEED_SERVO);
        CRServo SideServo = hardwareMap.get(CRServo.class, SIDE_SERVO);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        DistanceSensor RangeSensor = hardwareMap.get(DistanceSensor.class, "RangeSensor");
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
        feed.setPower(0);


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

                .stopAndAdd(new AutoMotorControl.ShooterAndFeederCombined(
                        shooter, intake,feed ,SideServo
                        ,RangeSensor,SHOOTER_VEL,SIDE_POWER,StartWaitTime,
                        WaitTime))
                .stopAndAdd(setMotorPower(intake, INTAKE_POWER))

                .setTangent(Math.toRadians(154.5))
                .stopAndAdd(setMotorPower(intake, INTAKE_POWER))
                .strafeToLinearHeading(new Vector2d(32, 23), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .lineToY(52)
                .stopAndAdd(setMotorPower(intake, 0.0))
                .strafeToLinearHeading(new Vector2d(51, 10), Math.toRadians(152.5))
                .stopAndAdd(setMotorVel(shooter, SHOOTER_VEL))

                .stopAndAdd(new AutoMotorControl.ShooterAndFeederCombined(
                        shooter, intake,feed ,SideServo
                        ,RangeSensor,SHOOTER_VEL,SIDE_POWER,StartWaitTime,
                        WaitTime))
                .stopAndAdd(setMotorPower(intake, INTAKE_POWER))
                .setTangent(Math.toRadians(152.5))
                .strafeToLinearHeading(new Vector2d(12, 15), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(12, 54), Math.toRadians(90))
                .build();

        Actions.runBlocking(all);

        // Safety park
        feed.setPower(0);
        shooter.setPower(0.0);
        intake.setPower(0.0);
    }
}