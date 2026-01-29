package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.feature.TelemetrySystem;
import org.firstinspires.ftc.teamcode.feature.vision.Vision;
import org.firstinspires.ftc.teamcode.part.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "AutoBlue")
@Config("AutoBlue")
public class AutoBlue extends LinearOpMode {
    MecanumDrive drive;
    AutoShooterManager manager;
    AutoIntake intake;
    Vision limelight;

    Pose2d current;

    VelConstraint intakeSpeed = new TranslationalVelConstraint(15.0);

    //    Pose2d startingPoint = new Pose2d(46.8038, -49.5795, Math.toRadians(-53.8742));
    Pose2d startingPoint = new Pose2d(54.2418, 44.7952, Math.toRadians(53.87));
    Pose2d shoot1 = new Pose2d(17.5323, 8.2525, Math.toRadians(45));
    Pose2d eat11 = new Pose2d(-30, 28, Math.toRadians(90));
    Pose2d eat12 = new Pose2d(-20, 52, Math.toRadians(90));
    Pose2d eat1_shoot2_transition = new Pose2d(-20, 18, Math.toRadians(90));
    Pose2d shoot2 = new Pose2d(9.7, 18, Math.toRadians(90));
    Pose2d shoot2_eat2_transistion = new Pose2d(9.7, 20, Math.toRadians(90));
    Pose2d eat2 = new Pose2d(13, 52, Math.toRadians(100));
    Pose2d shoot3 = shoot2;
    Pose2d eat31 = new Pose2d(-39, 20, Math.toRadians(90));
    Pose2d eat32 = new Pose2d(-39, 53, Math.toRadians(90));
    Pose2d shoot4 = new Pose2d(20, 20, Math.toRadians(90));

    public static boolean shooterOn = false;
    public static boolean closed = true;
    public static double hood = 0.0;
    public static boolean intakeOn = false;
    public static double turretAngle = 0.0;

    void stopAndTest() {
        while (true) {
            if (shooterOn) {
                this.manager.startMotor();
            } else {
                this.manager.stopMotor();
            }
            if (closed) {
                this.manager.closeStopper();
            } else {
                this.manager.openStopper();
            }
            if (intakeOn) {
                this.intake.intakeOn();
            } else {
                this.intake.intakeOff();
            }
            this.manager.lookAt(turretAngle, hood);
        }
    }

    void stopAndAlign() {
        while (true) {
            this.manager.turret.blockingAlign();
        }
    }

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, startingPoint);
        limelight = new Vision();
        intake = new AutoIntake(
                hardwareMap.get(DcMotorEx.class, "intake_motor")
        );
        manager = new AutoShooterManager(
                hardwareMap.get(DcMotorEx.class, "shooter_motor1"),
                hardwareMap.get(DcMotorEx.class, "shooter_motor2"),

                hardwareMap.get(Servo.class, "servo_stopper"),
                hardwareMap.get(Servo.class, "servo_hood"),

                hardwareMap.get(DcMotorEx.class, "turret"),
                limelight,
                intake
        );

        TelemetrySystem.init(telemetry);

        limelight.init(hardwareMap, telemetry);
        manager.init();
        intake.init();

        waitForStart();

        limelight.start();
        limelight.setPipeline(Constants.PIPELINE.BLUE_GOAL);
        manager.start();
        manager.startMotor();

        manager.lookAt(0.2, 0.75);
        current = startingPoint;
        Actions.runBlocking(
                drive.actionBuilder(current)
                        .strafeToLinearHeading(shoot1.position, shoot1.heading)
                        .build()
        );


        intake.intakeOn();
        manager.blockingShoot(4.4, 0); // I have no idea what the units are, but 4.4 works.

        current = drive.localizer.getPose();
        Actions.runBlocking(
                drive.actionBuilder(current)
                        .strafeToLinearHeading(
                                shoot2_eat2_transistion.position,
                                shoot2_eat2_transistion.heading
                        )
                        .strafeToLinearHeading(
                                eat2.position,
                                eat2.heading,
                                intakeSpeed
                        )
                        .build()
        );


        manager.lookAt(0, 0.75);
        current = drive.localizer.getPose();
        Actions.runBlocking(
                drive.actionBuilder(current)
                        .strafeToLinearHeading(shoot1.position, shoot1.heading)
                        .build()
        );
        manager.blockingShoot(4.4, 0);

//        current = drive.localizer.getPose();
//        Actions.runBlocking(
//                drive.actionBuilder(current)
//                        .strafeToLinearHeading(eat31.position, eat31.heading)
//                        .waitSeconds(0.2)
//                        .strafeToLinearHeading(
//                                eat32.position,
//                                eat32.heading,
//                                intakeSpeed
//                        )
//                        .waitSeconds(0.2)
//                        .strafeToLinearHeading(shoot4.position, shoot4.heading)
//                        .build()
//        );
//        stopAndTest();

        current = drive.localizer.getPose();
        Actions.runBlocking(
                drive.actionBuilder(current)
                        .strafeToLinearHeading(eat11.position, eat11.heading)
                        .build()
        );
    }
}