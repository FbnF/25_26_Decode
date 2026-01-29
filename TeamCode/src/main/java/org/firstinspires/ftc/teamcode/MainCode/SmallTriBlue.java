package org.firstinspires.ftc.teamcode.MainCode;

import static org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.setMotorPower;
import static org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.setMotorVel;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl;
import org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.ShooterAndFeederAction;
import org.firstinspires.ftc.teamcode.MecanumDrive;
@Config
@Autonomous(name="SmallTriBlue", group="Auto")
public class SmallTriBlue extends LinearOpMode {

    // RC config names
    private static final String FEED_SERVO   = "feedServo";
    private static final String INTAKE_MOTOR = "IntakeMotor";
    private static final String LAUNCH_MOTOR = "LaunchMotor";
    private static final String SIDE_SERVO = "sideServo";
    private static final String LIMELIGHT = "Limelight";

    //private static final String VOLTAGE_SENSOR = "VoltageSensor";

    // Tunables
    public static final double INTAKE_POWER  = 0.73;
    public static double SHOOTER_POWER = 0.74;

    public static double SHOOTER_VEL = 1755;

    public static double WaitTime = 10.5;
    public static double StartWaitTime = 2;
    public static double SIDE_POWER = -0.1;


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(60, -16, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        DcMotor intake        = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);
        DcMotorEx shooter     = (DcMotorEx) hardwareMap.get(DcMotor.class, LAUNCH_MOTOR);
        CRServo feed            = hardwareMap.get(CRServo.class, FEED_SERVO);
        CRServo SideServo = hardwareMap.get(CRServo.class, SIDE_SERVO);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        DistanceSensor RangeSensor = hardwareMap.get(DistanceSensor.class, "RangeSensor");
        Limelight3A LimeLight = hardwareMap.get(Limelight3A.class, LIMELIGHT);
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
        telemetry.update();

        waitForStart();
        telemetry.addData("SHOOTER_VELOCITY", shooter.getVelocity());
        telemetry.update();
        if (isStopRequested()) return;

        Action all = drive.actionBuilder(startPose)
                .stopAndAdd(setMotorPower(intake, 0.0))
                .stopAndAdd(setMotorVel(shooter, SHOOTER_VEL))
                .strafeToLinearHeading(new Vector2d(52, -10), Math.toRadians(201))
                .stopAndAdd(AutoMotorControl.setMotorPower(intake, 0.7))
                .stopAndAdd(new AutoMotorControl.ShooterAndFeederCombined(
                        shooter, intake,feed ,SideServo
                        ,RangeSensor,SHOOTER_VEL,SIDE_POWER,StartWaitTime,
                        WaitTime))
                .stopAndAdd(setMotorPower(intake, INTAKE_POWER))
                .setTangent(Math.toRadians(204))
                .strafeToLinearHeading(new Vector2d(30, -26), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .lineToY(-48)
                .lineToY(-55)
                // .lineToY(-36)
                .stopAndAdd(setMotorPower(intake, 0.0))
                .stopAndAdd(setMotorVel(shooter, SHOOTER_VEL))
                .strafeToLinearHeading(new Vector2d(52, -14), Math.toRadians(203))


                .stopAndAdd(new AutoMotorControl.ShooterAndFeederCombined(
                        shooter, intake,feed ,SideServo
                        ,RangeSensor,SHOOTER_VEL,SIDE_POWER,StartWaitTime,
                        WaitTime))
                .stopAndAdd(setMotorPower(intake, INTAKE_POWER))

                //.turn(Math.toRadians(ANGLE_OF_TURN))
                .setTangent(Math.toRadians(215))
                .strafeToLinearHeading(new Vector2d(40, -15), Math.toRadians(270))
                .build();

        Actions.runBlocking(all);

        // Safety park
        feed.setPower(0);
        shooter.setPower(0.0);
        intake.setPower(0.0);
    }
}