package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.feature.Schedule;
import org.firstinspires.ftc.teamcode.feature.SmartGamepad;
import org.firstinspires.ftc.teamcode.feature.TelemetrySystem;
import org.firstinspires.ftc.teamcode.part.Constants;

import static org.firstinspires.ftc.teamcode.part.Constants.*;

import org.firstinspires.ftc.teamcode.part.NewDrive;
import org.firstinspires.ftc.teamcode.part.Part;
import org.firstinspires.ftc.teamcode.part.intake.Intake;
import org.firstinspires.ftc.teamcode.part.shooter.Shooter;
import org.firstinspires.ftc.teamcode.part.Turret;
import org.firstinspires.ftc.teamcode.feature.vision.Vision;

import java.util.List;

@TeleOp(name = "Red", group = "Match")
public class TeleOpRed extends OpMode {

    private SmartGamepad smartGamepad1, smartGamepad2;
    private final Intake intake = new Intake();
    private final NewDrive drive = new NewDrive();
    private final Vision vision = new Vision();
    private final Turret turret = new Turret(vision);
    private final Shooter shooter = new Shooter();

    boolean hoodAngelStatic;

    FtcDashboard dashboard;

    private Part[] parts = new Part[]{};

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        parts = new Part[]{intake, turret, shooter, drive};
        for (Part part: parts) {
            part.init(hardwareMap, telemetry);
        }
        vision.init(hardwareMap, telemetry);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        turret.setTurret_offset_red(true);
        hoodAngelStatic = false;

        smartGamepad1 = new SmartGamepad(gamepad1);
        smartGamepad2 = new SmartGamepad(gamepad2);

        TelemetrySystem.init(telemetry);
        Schedule.init();
    }

    @Override
    public void start() {
        super.start();
        for (Part part: parts) {
            part.start();
        }
        vision.start();
        vision.setPipeline(PIPELINE.RED_GOAL);


    }

    //    double lastLoopTime = 0;
    @Override
    public void loop() {

//        double currentLoopTime = System.nanoTime();
//        double loopHz = 1000000000 / (currentLoopTime - lastLoopTime);
//        lastLoopTime = currentLoopTime;

        // player 1 : Driving + Intake

        double rx = 0;
        if (smartGamepad1.buttonRightBumper().isDown()) rx = -DRIVE_ROTATION_SPEED;
        else if (smartGamepad1.buttonLeftBumper().isDown()) rx = DRIVE_ROTATION_SPEED;
        drive.setDrivePowers(
                -  Math.pow(smartGamepad1.triggerLeftStickX().getValue(), 3),
                - Math.pow(smartGamepad1.triggerLeftStickY().getValue(), 3),
                rx
        );

        if (smartGamepad1.buttonDPadUp().isHeld()){
            drive.setDrivePowers(0,5,0);
        }
        if (smartGamepad1.buttonDPadDown().isHeld()){
            drive.setDrivePowers(0,-5,0);
        }
        if (smartGamepad1.buttonDPadLeft().isHeld()){
            drive.setDrivePowers(5,0,0);
        }
        if (smartGamepad1.buttonDPadRight().isHeld()){
            drive.setDrivePowers(-5,0,0);
        }



        if (smartGamepad1.buttonX().isPressed()) {
            intake.cmdRun();
        }
        else if (smartGamepad1.buttonY().isPressed()){
            intake.cmdStop();
        }

        // player 2 : Shooter + Turret

        if (smartGamepad2.buttonLeftBumper().isDown()){
            turret.turnOffVision();
            turret.changeTargetPos(TURRET_ROTATION_VEL);
        } else if (smartGamepad2.buttonRightBumper().isDown()) {
            turret.turnOffVision();
            turret.changeTargetPos(-TURRET_ROTATION_VEL);
        }
        if (smartGamepad2.triggerLeftTrigger().isHeld()){
            turret.turnOffVision();
            turret.changeTargetPos(TURRET_ROTATION_VEL_FASTER);
        } else if (smartGamepad2.triggerRightTrigger().isHeld()){
            turret.turnOffVision();
            turret.changeTargetPos(-TURRET_ROTATION_VEL_FASTER);
        }

        if (smartGamepad2.buttonDPadUp().isHeld()) {
            shooter.cmdStopperOpen();
        } if (smartGamepad2.buttonDPadDown().isHeld()) {
            shooter.cmdStopperClose();
        }

        if (smartGamepad2.buttonX().isPressed()){
            turret.toggleVision();
        }

        if (smartGamepad2.buttonY().isPressed()){
            shooter.cmdShooterRun();
        }
        if (smartGamepad2.buttonA().isPressed()){
            shooter.cmdShooterStop();
        }

        if (smartGamepad2.buttonB().isPressed()){
            hoodAngelStatic = !hoodAngelStatic;
        }


        vision.update();
        // update parts
        for (Part part : parts){
            part.update();
        }

        if (hoodAngelStatic){
            shooter.setAngle(0.5);
        }
        else {
            shooter.cmdSetAngleByDist(vision.getDistance());
        }



        TelemetrySystem.addClassData("Vision", "Detected", vision.tagDetected());


        Schedule.update();

        TelemetrySystem.update();

        // loop의 맨 마지막에 실행!
        smartGamepad1.update();
        smartGamepad2.update();
    }


    @Override
    public void stop() {
        for (Part part : parts){
            part.stop();
        }
        vision.stop();
        Schedule.stop();
        super.stop();
    }



}