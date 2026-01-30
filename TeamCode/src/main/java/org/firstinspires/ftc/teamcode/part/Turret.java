package org.firstinspires.ftc.teamcode.part;

import static org.firstinspires.ftc.teamcode.part.Constants.*;

import android.os.ParcelUuid;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.feature.PID;
import org.firstinspires.ftc.teamcode.feature.TelemetrySystem;
import org.firstinspires.ftc.teamcode.feature.vision.Vision;


public class Turret implements Part {

    DcMotorEx motor;
    DcMotorEx encoder;
    PID pidController;

    int targetPos; // keeps the target position set on runPIDToPosition
    boolean usingVision;
    Vision vision;
    double turret_offset;

//    double position;

    public Turret(Vision vision) {
        this.vision = vision;
    }

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        motor = hardwareMap.get(DcMotorEx.class, "turret");
        encoder = hardwareMap.get(DcMotorEx.class, "turret");

        pidController = new PID(TURRET_PID_VISION_P, TURRET_PID_VISION_I, TURRET_PID_VISION_D);
    }

    @Override
    public void start() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        usingVision = false;

//        position = 0;
    }

    public void setTurret_offset_red(boolean red){
        if (red) {
            turret_offset = TURRET_PID_VISION_OFFSET_RED;
        }
        else {
            turret_offset = TURRET_PID_VISION_OFFSET_BLUE;
        }
    }

    @Override
    public void update() {
        double pos = encoder.getCurrentPosition();
        TelemetrySystem.addClassData("TURRET", "position", pos);
        TelemetrySystem.addClassData("TURRET", "targetPosition", targetPos);
        TelemetrySystem.addClassData("TURRET", "usingVision", usingVision);

        if (pos < TURRET_LEFT_END || TURRET_RIGHT_END < pos) {
            usingVision = false;
        }
        targetPos = Math.max(TURRET_LEFT_END, Math.min(TURRET_RIGHT_END, targetPos));

        if (usingVision) {
            runPIDWithVision();
            targetPos = encoder.getCurrentPosition();
        } else {
            runPIDToPosition(targetPos);
        }




    }

    @Override
    public void stop() {
        motor.setPower(0);
    }

    public void runPIDWithVision() {
        pidController.updatePID(TURRET_PID_VISION_P, TURRET_PID_VISION_I, TURRET_PID_VISION_D);
        double currentAngle;

        if (vision.tagDetected()) {
            currentAngle = vision.getAngle();
        } else {
            motor.setPower(0);
            return;
        }
        if (motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        double targetAngle = turret_offset;
        double error = currentAngle - targetAngle;
        if (Math.abs(error) < TURRET_PID_THRESHOLD) {
            error = 0;
        }
        double pidOutput = pidController.update(error, -TURRET_MAXIMUM_POWER, TURRET_MAXIMUM_POWER);

        TelemetrySystem.addClassData("TURRET", "error", error);
        TelemetrySystem.addClassData("TURRET", "pid", pidOutput);
        TelemetrySystem.addClassData("TURRET", "mode", "with vision");

        motor.setPower(pidOutput);
        targetPos = encoder.getCurrentPosition();
    }

    public void runPIDToPosition(int targetPos) {

        motor.setTargetPosition(targetPos);
        if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        motor.setPower(0.8); // 파워는 모드 변경 시 한 번만 줘도 유지됨 (필요하면 밖으로 빼도 무방)
    }

    public void toggleVision() {
        if (usingVision) turnOffVision();
        else turnOnVision();
    }

    public void turnOnVision() {
        usingVision = true;
    }

    public void turnOffVision() {
        usingVision = false;
    }

    public void changeTargetPos(int v) {
        targetPos += v;
    }
}
