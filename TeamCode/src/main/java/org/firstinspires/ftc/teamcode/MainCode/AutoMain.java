package org.firstinspires.ftc.teamcode.MainCode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket; // if you have the dashboard dep
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Action;

@Autonomous(name="AutoMain", group="Main")
public class AutoMain extends LinearOpMode {

    // Small helper Action that sets a motor power once and immediately completes
    private static Action motorPower(DcMotor m, double p) {
        return (TelemetryPacket packet) -> { m.setPower(p); return true; };
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Motors you want to toggle during "waits"
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        DcMotorEx launchMotor = hardwareMap.get(DcMotorEx.class, "LaunchMotor"); // unused here, just leaving as-is
        intakeMotor.setPower(0);

        Action all = drive.actionBuilder(startPose)

                // --- Leg 1 ---
                .splineTo(new Vector2d(0, 0), Math.toRadians(135))

                .stopAndAdd(motorPower(launchMotor, 1.0))   // ON
                .waitSeconds(2)
                .stopAndAdd(motorPower(launchMotor, 0.0))   // OFF

                .setTangent(Math.toRadians(90))
                .afterDisp(0.0, motorPower(intakeMotor, 1.0))
                .splineToLinearHeading(new Pose2d(-11, 33, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(45)
                .lineToY(33)
                .afterDisp(0.0, motorPower(intakeMotor, 0.0))

                // --- Leg 2 ---
                .splineTo(new Vector2d(0, 0), Math.toRadians(-40))
                .stopAndAdd(motorPower(launchMotor, 1.0))   // ON
                .waitSeconds(2)
                .stopAndAdd(motorPower(launchMotor, 0.0))   // OFF

                .setTangent(Math.toRadians(90))
                .afterDisp(0.0, motorPower(intakeMotor, 1.0))
                .splineToLinearHeading(new Pose2d(11, 33, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(45)
                .lineToY(33)
                .afterDisp(0.0, motorPower(intakeMotor, 0.0))

                // --- Leg 3 ---
                .splineTo(new Vector2d(0, 0), Math.toRadians(-40))
                .stopAndAdd(motorPower(launchMotor, 1.0))   // ON
                .waitSeconds(2)
                .stopAndAdd(motorPower(launchMotor, 0.0))   // OFF

                .setTangent(Math.toRadians(90))
                .afterDisp(0.0, motorPower(intakeMotor, 1.0))
                .splineToLinearHeading(new Pose2d(34, 33, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(45)
                .lineToY(33)
                .afterDisp(0.0, motorPower(intakeMotor, 0.0))

                // --- Leg 4 ---
                .splineTo(new Vector2d(0, 0), Math.toRadians(-40))
                .stopAndAdd(motorPower(launchMotor, 1.0))   // ON
                .waitSeconds(2)
                .stopAndAdd(motorPower(launchMotor, 0.0))   // OFF

                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(all);

        // safety
        intakeMotor.setPower(0);
        if (launchMotor != null) launchMotor.setPower(0);
    }
}