package org.firstinspires.ftc.teamcode.part;

import static org.firstinspires.ftc.teamcode.part.Constants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.feature.PID;
import org.firstinspires.ftc.teamcode.feature.TelemetrySystem;
import org.firstinspires.ftc.teamcode.part.vision.Vision;


public class Turret implements Part {

    DcMotorEx motor;
    DcMotorEx encoder;
    PID pidController;
    PID pidControllerWithEncoder;
    boolean isWrappingNow; // 한바퀴 회전중인지 - true 일때 대기
    boolean finishedWrapping; // 한바퀴 회전 후 - Limit 넘었을때 행동 결정
    double target_pos; // keeps the target position set on runPIDToPosition

    Vision vision;

    public Turret(Vision vision){
        this.vision = vision;
    }

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        motor = hardwareMap.get(DcMotorEx.class, "turret");
        encoder = hardwareMap.get(DcMotorEx.class, "turret");

        pidController = new PID(TURRET_PID_VISION_P, TURRET_PID_VISION_I, TURRET_PID_VISION_D);
        pidControllerWithEncoder = new PID(TURRET_PID_POSITION_P, TURRET_PID_POSITION_I, TURRET_PID_POSITION_D);
    }

    @Override
    public void start() {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        isWrappingNow = false;
        finishedWrapping = false;

    }


    @Override
    public void update() {
        double pos = encoder.getCurrentPosition();
        TelemetrySystem.addClassData("TURRET", "position", pos);

        // LIMIT : 넘으면 반대로 회전 +
        //      대신 회전한 직후에는 LIMIT 넘어도 됨
        //      --> 경계 부근에서 진동하는 상황 막기
        // END : 물리적 한계 --> 무조건 반대로 회전

        // (이동중 == true) :
            // (pid 꺼져있음)
            // (target_pos에 도착) :
                // 이동중 = false
        // (pos < LEFT_END or RIGHT_END < pos) :
            // 즉시 한바퀴 회전 + pid 끄기
        // LEFT_LIMIT < pos < RIGHT_LIMIT :
            // 그냥 비전 기반 pid
            // 한바퀴 회전 후 = false
        // (pos < LEFT_LIMIT or RIGHT_LIMIT < pos) and (한바퀴 회전 후 == false) :
            // 즉시 한바퀴 회전 + pid 끄기
            // 한바퀴 회전후 = true
        // (pos < LEFT_LIMIT or RIGHT_LIMIT < pos) and (한바퀴 회전 후 == true) :
            // 그냥 비전 기반 pid

        if (isWrappingNow){
            runPIDToPosition(target_pos);
            if (Math.abs(pos - target_pos) < TURRET_DIFF_THRESHOLD){
                isWrappingNow = false;
            }
            return;
        }
        if (pos < TURRET_LEFT_END || TURRET_RIGHT_END < pos){
            if (pos < TURRET_LEFT_END) runPIDToPosition(pos + TURRET_ONE_REV_TICKS);
            else                       runPIDToPosition(pos - TURRET_ONE_REV_TICKS);
            isWrappingNow = true;
        }
        else if (TURRET_LEFT_LIMIT < pos && pos < TURRET_RIGHT_LIMIT) {
            runPIDWithVision();
            finishedWrapping = false;
        }
        else if ((pos < TURRET_LEFT_LIMIT || pos > TURRET_RIGHT_LIMIT) && (finishedWrapping == false)) {
            if (pos < TURRET_LEFT_LIMIT) runPIDToPosition(pos + TURRET_ONE_REV_TICKS);
            else                         runPIDToPosition(pos - TURRET_ONE_REV_TICKS);
            isWrappingNow = true;
            finishedWrapping = true;
        }
        else {          // <==> else if ((pos < TURRET_LEFT_LIMIT || pos > TURRET_RIGHT_LIMIT) && (finishedWrapping == true)) {
            runPIDWithVision();
        }


    }

    @Override
    public void stop() {
        motor.setPower(0);
    }

    public void runPIDWithVision(){
        double currentAngle = vision.getPos()[0];
        double error = currentAngle - 0;
        TelemetrySystem.addClassData("TURRET", "error", error);
        double pidOutput = pidController.update(error, -TURRET_MAXIMUM_POWER, TURRET_MAXIMUM_POWER);
        TelemetrySystem.addClassData("TURRET", "pid", pidOutput);
        TelemetrySystem.addClassData("TURRET", "mode", "with vision");

        motor.setPower(pidOutput);
    }

    public void runPIDToPosition(double targetAngle){
        double currentAngle = encoder.getCurrentPosition();
        target_pos = targetAngle;
        double error = currentAngle - targetAngle;
        TelemetrySystem.addClassData("TURRET", "error", error);
        double pidOutput = pidControllerWithEncoder.update(error, -TURRET_MAXIMUM_POWER, TURRET_MAXIMUM_POWER);
        TelemetrySystem.addClassData("TURRET", "mode", "run to position");

        motor.setPower(pidOutput);
    }
}
