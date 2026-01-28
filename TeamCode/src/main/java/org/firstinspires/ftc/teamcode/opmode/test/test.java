package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.feature.Schedule;
import org.firstinspires.ftc.teamcode.feature.SmartGamepad;
import org.firstinspires.ftc.teamcode.feature.TelemetrySystem;
import org.firstinspires.ftc.teamcode.part.Constants;
import org.firstinspires.ftc.teamcode.part.Drive;
import org.firstinspires.ftc.teamcode.part.Part;
import org.firstinspires.ftc.teamcode.part.intake.Intake;
import org.firstinspires.ftc.teamcode.part.shooter.Shooter;
import org.firstinspires.ftc.teamcode.part.Turret;
import org.firstinspires.ftc.teamcode.part.shooter.ShooterState;
import org.firstinspires.ftc.teamcode.feature.vision.Vision;

@TeleOp(name = "TeleOp")
public class test extends OpMode {

    private SmartGamepad smartGamepad1, smartGamepad2;
    private final Intake intake = new Intake();
    private final Drive drive = new Drive();
    private final Vision vision = new Vision();
    private final Turret turret = new Turret(vision);
    private final Shooter shooter = new Shooter(vision);

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
        vision.setPipeline(Constants.PIPELINE.RED_GOAL);
    }


    @Override
    public void loop() {
        handleDrive();

        // INTAKE
        if (smartGamepad1.buttonX().isPressed()) {
            TelemetrySystem.addClassData("GAMEPAD", "X", "clicked");
            switch (intake.state) {
                case IDLE:
                    intake.cmdRun();
                    break;
                case RUN:
                    intake.stop();
                    break;
                default:
                    intake.stop();
                    break;
            }
        }

        if (smartGamepad1.buttonY().isPressed()){
            if (shooter.shooterState == ShooterState.RUN) {
                shooter.cmdShooterStop();
            }
            else{
                shooter.cmdShooterRun();
            }
        }



        // SHOOTER STOPPER TOGGLE
        if (smartGamepad2.buttonB().isPressed()) {
            shooter.cmdStopperClose();
        }
        else if (smartGamepad2.buttonA().isPressed()) {
            shooter.cmdStopperOpen();
        }

        if (smartGamepad2.buttonX().isPressed()){
            turret.toggleVision();
        }
        if (smartGamepad2.buttonLeftBumper().isPressed()){
            turret.changeTargetPos(-50);
        } else if (smartGamepad2.buttonRightBumper().isPressed()) {
            turret.changeTargetPos(50);
        }


        vision.update();
        // update parts
        for (Part part : parts){
            part.update();
        }

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
        super.stop();
    }


    // commands
    // RoadRunner 로 대체 후, 제거 예정
    private void handleDrive() {
        double power = Constants.DRIVE_POWER;

        if (smartGamepad1.buttonDPadUp().isDown()) {
            drive.motorLF.setPower(power);
            drive.motorLR.setPower(power);
            drive.motorRF.setPower(power);
            drive.motorRR.setPower(power);
        }
        else if (smartGamepad1.buttonDPadDown().isDown()) {
            drive.motorLF.setPower(-power);
            drive.motorLR.setPower(-power);
            drive.motorRF.setPower(-power);
            drive.motorRR.setPower(-power);
        }
        else if (smartGamepad1.buttonDPadRight().isDown()) {
            drive.motorLF.setPower(power);
            drive.motorLR.setPower(-power);
            drive.motorRF.setPower(-power);
            drive.motorRR.setPower(power);
        }
        else if (smartGamepad1.buttonDPadLeft().isDown()) {
            drive.motorLF.setPower(-power);
            drive.motorLR.setPower(power);
            drive.motorRF.setPower(power);
            drive.motorRR.setPower(-power);
        }
        else if (smartGamepad1.buttonLeftBumper().isDown()) {
            drive.motorLF.setPower(-power);
            drive.motorLR.setPower(-power);
            drive.motorRF.setPower(power);
            drive.motorRR.setPower(power);
        }
        else if (smartGamepad1.buttonRightBumper().isDown()) {
            drive.motorLF.setPower(power);
            drive.motorLR.setPower(power);
            drive.motorRF.setPower(-power);
            drive.motorRR.setPower(-power);
        }
        else {
            drive.motorLF.setPower(0);
            drive.motorLR.setPower(0);
            drive.motorRF.setPower(0);
            drive.motorRR.setPower(0);
        }
    }

}