package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.feature.SmartGamepad;
import org.firstinspires.ftc.teamcode.feature.TelemetrySystem;
import org.firstinspires.ftc.teamcode.part.NewDrive;
import org.firstinspires.ftc.teamcode.part.Part;
import org.firstinspires.ftc.teamcode.part.intake.Intake;
import org.firstinspires.ftc.teamcode.part.shooter.Shooter;
import org.firstinspires.ftc.teamcode.part.Turret;
import org.firstinspires.ftc.teamcode.part.shooter.ShooterState;
import org.firstinspires.ftc.teamcode.feature.vision.Vision;
import static org.firstinspires.ftc.teamcode.part.Constants.*;

@TeleOp(name = "TeleOp")
public class TeleOpRed extends OpMode {

    private SmartGamepad smartGamepad1, smartGamepad2;
    private final Intake intake = new Intake();
    private final NewDrive drive = new NewDrive();
    private final Vision vision = new Vision();
    private final Turret turret = new Turret(vision);
    private final Shooter shooter = new Shooter(vision);

    private Part[] parts = new Part[]{};

    @Override
    public void init() {
        parts = new Part[]{intake, turret, shooter, drive};
        for (Part part: parts) {
            part.init(hardwareMap, telemetry);
        }
        vision.init(hardwareMap, telemetry);

        smartGamepad1 = new SmartGamepad(gamepad1);
        smartGamepad2 = new SmartGamepad(gamepad2);

        TelemetrySystem.init(telemetry);
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

    @Override
    public void loop() {
//        handleDrive();

        // [read gamepads Signal]
        // INTAKE
        if (smartGamepad2.buttonX().isPressed()) {
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

        // SHOOTER STOPPER TOGGLE
        if (smartGamepad2.buttonB().isPressed()) {
            shooter.cmdStopperOpen();
        }
        if (smartGamepad2.buttonA().isPressed()){
            shooter.cmdStopperClose();
        }

        if (smartGamepad2.buttonY().isPressed()){
            if (shooter.shooterState == ShooterState.RUN) {
                shooter.cmdShooterRun();
                shooter.shooterState = ShooterState.STOP;
            }
            else{
                shooter.cmdShooterStop();
                shooter.shooterState = ShooterState.RUN;
            }
        }

        double x,y, rx;
        x = -smartGamepad2.triggerLeftStickX().getValue();
        y = -smartGamepad2.triggerLeftStickY().getValue();
        rx = smartGamepad2.triggerRightStickX().getValue();
        drive.setDrivePowers(x,y,rx);


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


    // RoadRunner 로 대체 후, 제거 예정
//    private void handleDrive() {
//        double power = Constants.DRIVE_POWER;
//
//        if (smartGamepad2.buttonDPadUp().isDown()) {
//            drive.motorLF.setPower(power);
//            drive.motorLR.setPower(power);
//            drive.motorRF.setPower(power);
//            drive.motorRR.setPower(power);
//        }
//        else if (smartGamepad2.buttonDPadDown().isDown()) {
//            drive.motorLF.setPower(-power);
//            drive.motorLR.setPower(-power);
//            drive.motorRF.setPower(-power);
//            drive.motorRR.setPower(-power);
//        }
//        else if (smartGamepad2.buttonDPadRight().isDown()) {
//            drive.motorLF.setPower(power);
//            drive.motorLR.setPower(-power);
//            drive.motorRF.setPower(-power);
//            drive.motorRR.setPower(power);
//        }
//        else if (smartGamepad2.buttonDPadLeft().isDown()) {
//            drive.motorLF.setPower(-power);
//            drive.motorLR.setPower(power);
//            drive.motorRF.setPower(power);
//            drive.motorRR.setPower(-power);
//        }
//        else if (smartGamepad2.buttonLeftBumper().isDown()) {
//            // CCW Rotate
//            drive.motorLF.setPower(-power);
//            drive.motorLR.setPower(-power);
//            drive.motorRF.setPower(power);
//            drive.motorRR.setPower(power);
//        }
//        else if (smartGamepad2.buttonRightBumper().isDown()) {
//            // CW Rotate
//            drive.motorLF.setPower(power);
//            drive.motorLR.setPower(power);
//            drive.motorRF.setPower(-power);
//            drive.motorRR.setPower(-power);
//        }
//        else {
//            drive.motorLF.setPower(0);
//            drive.motorLR.setPower(0);
//            drive.motorRF.setPower(0);
//            drive.motorRR.setPower(0);
//        }
//    }

}