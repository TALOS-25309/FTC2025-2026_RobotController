package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name="AutoJazzim")
public class Jazzim extends LinearOpMode {
    MecanumDrive drive;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))
                        .strafeToLinearHeading(new Vector2d(25, 0), 0)
                        .build()
        );
    }
}
