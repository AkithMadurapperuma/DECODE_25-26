package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;
// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Autonomous(name = "TestPathing", group = "23393 Auto")
public class TestPathing extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //starting Pose
        Pose2d beginPose = new Pose2d(new Vector2d(0, 63), Math.toRadians(270));

        // create RR drive obj
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        //Autonomous Path
        Action path = drive.actionBuilder(beginPose)
                .splineToSplineHeading(new Pose2d(40, 0, Math.toRadians(180)), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(40, -48), Math.toRadians(0))
                .lineToXLinearHeading(0, Math.toRadians(90))
                .build();



        Actions.runBlocking(
                new SequentialAction(path)
        );
    }
}