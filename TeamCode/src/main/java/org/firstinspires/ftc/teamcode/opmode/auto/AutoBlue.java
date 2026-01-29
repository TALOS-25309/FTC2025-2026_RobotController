package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.feature.vision.Vision;
import org.firstinspires.ftc.teamcode.part.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "AutoBlue")
public class AutoBlue extends LinearOpMode {
    MecanumDrive drive;
    AutoShooterManager manager;
    AutoIntake intake;
    Vision limelight;

    Pose2d current;


    VelConstraint intakeSpeed = new TranslationalVelConstraint(25.0);

//    Pose2d startingPoint = new Pose2d(46.8038, -49.5795, Math.toRadians(-53.8742));
    Pose2d startingPoint = new Pose2d(54.2418, 44.7952, Math.toRadians(53.87));
    Pose2d shoot1 = new Pose2d(17.5323, 8.2525, Math.toRadians(45));
    Pose2d eat11 = new Pose2d(-13, 18, Math.toRadians(100));
    Pose2d eat12 = new Pose2d(-13, 50.5, Math.toRadians(90));
    Pose2d eat1_shoot2_transition = new Pose2d(-14.5, 20, Math.toRadians(90));
    Pose2d shoot2 = new Pose2d(11, 20, Math.toRadians(90));
    Pose2d eat2 = new Pose2d(11, 52, Math.toRadians(90));
    Pose2d shoot3 = shoot2;
    Pose2d eat31 = new Pose2d(-39, 20, Math.toRadians(90));
    Pose2d eat32 = new Pose2d(-39, 53, Math.toRadians(90));
    Pose2d shoot4 = new Pose2d(20, 20, Math.toRadians(90));
    //not fixed


    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, startingPoint);
        intake = new AutoIntake(
                hardwareMap.get(DcMotorEx.class, "intake_motor")
        );
        limelight = new Vision();
        manager = new AutoShooterManager(
                hardwareMap.get(DcMotorEx.class, "shooter_motor1"),
                hardwareMap.get(DcMotorEx.class, "shooter_motor2"),

                hardwareMap.get(Servo.class, "servo_stopper"),
                hardwareMap.get(Servo.class, "servo_hood"),

                hardwareMap.get(DcMotorEx.class, "turret"),
                limelight,
                intake
        );

        limelight.init(hardwareMap, telemetry);
        manager.init();
        intake.init();

        waitForStart();

        limelight.start();
        limelight.setPipeline(Constants.PIPELINE.BLUE_GOAL);
        manager.start();

        current = startingPoint;
        manager.lookAt(0, 0);
        Actions.runBlocking(
                drive.actionBuilder(current)
                        .strafeToLinearHeading(shoot1.position, shoot1.heading)
                        .build()
        );
//        manager.shoot();
//        while (manager.isShooting()) manager.update();

        sleep(2000); // For now
        current = drive.localizer.getPose();
        Actions.runBlocking(
                drive.actionBuilder(current)
                        .strafeToLinearHeading(eat11.position, eat11.heading)
                        .build()
        );
        intake.intakeOn();
        current = drive.localizer.getPose();

        Actions.runBlocking(
                drive.actionBuilder(current)
                        .strafeToLinearHeading(
                                eat12.position,
                                eat12.heading,
                                intakeSpeed
                        )
                        .build()
        );
        sleep(2000);

        current = drive.localizer.getPose();
        Actions.runBlocking(
                drive.actionBuilder(current)
                        .strafeToLinearHeading(eat1_shoot2_transition.position, eat1_shoot2_transition.heading)
                        .waitSeconds(0.2)
                        .strafeToLinearHeading(shoot2.position, shoot2.heading)
                        .build()
        );
        sleep(5000);

        current = drive.localizer.getPose();
        Actions.runBlocking(
                drive.actionBuilder(current)
                        .strafeToLinearHeading(
                                eat2.position,
                                eat2.heading,
                                intakeSpeed
                        )
                        .build()
        );

        current = drive.localizer.getPose();
        Actions.runBlocking(
                drive.actionBuilder(current)
                        .strafeToLinearHeading(shoot3.position, shoot3.heading)
                        .build()
        );
        sleep(2000); // Shoot

        current = drive.localizer.getPose();
        Actions.runBlocking(
                drive.actionBuilder(current)
                        .strafeToLinearHeading(eat31.position, eat31.heading)
                        .waitSeconds(0.2)
                        .strafeToLinearHeading(
                                eat32.position,
                                eat32.heading,
                                intakeSpeed
                        )
                        .waitSeconds(0.2)
                        .strafeToLinearHeading(shoot4.position, shoot4.heading)
                        .build()
        );
    }
}
