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

    double targetPos; // keeps the target position set on runPIDToPosition
    boolean usingVision;
    Vision vision;

//    double position;

    public Turret(Vision vision){
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

    boolean limitOverStop = false;

    @Override
    public void update() {
        double pos = encoder.getCurrentPosition();
        TelemetrySystem.addClassData("TURRET", "position", pos);
        TelemetrySystem.addClassData("TURRET", "targetPosition", targetPos);
        TelemetrySystem.addClassData("TURRET", "usingVision", usingVision);

        if (pos < TURRET_LEFT_END || TURRET_RIGHT_END < pos) {
            usingVision = false;
            limitOverStop = false;
        }
        targetPos = Math.max(TURRET_LEFT_END, Math.min(TURRET_RIGHT_END, targetPos));

        if (usingVision){
            runPIDWithVision();
        }
        else{
            runPIDToPosition(targetPos);
        }

        if (pos < TURRET_LEFT_END && !limitOverStop){
            motor.setPower(0);
            limitOverStop = true;
            targetPos = TURRET_LEFT_END/2;
            TelemetrySystem.addClassData("TURRET", "EMERGENCY STOP", true);
        }
        else if (pos > TURRET_RIGHT_END && !limitOverStop){
            motor.setPower(0);
            limitOverStop = true;
            targetPos = TURRET_LEFT_END/2;
            TelemetrySystem.addClassData("TURRET", "EMERGENCY STOP", true);
        }

    }

    @Override
    public void stop() {
        motor.setPower(0);
    }

    public void runPIDWithVision(){
        pidController.updatePID(TURRET_PID_VISION_P, TURRET_PID_VISION_I, TURRET_PID_VISION_D);
        double currentAngle;

        if (vision.tagDetected()){
            currentAngle = vision.getAngle();
        }
        else {
            return;
        }
        double error = currentAngle - 0;
        if (Math.abs(error) < TURRET_PID_THRESHOLD){
            error = 0;
        }
        double pidOutput = pidController.update(error, -TURRET_MAXIMUM_POWER, TURRET_MAXIMUM_POWER);

        TelemetrySystem.addClassData("TURRET", "error", error);
        TelemetrySystem.addClassData("TURRET", "pid", pidOutput);
        TelemetrySystem.addClassData("TURRET", "mode", "with vision");

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(pidOutput);
        targetPos = encoder.getCurrentPosition();
    }

    public void runPIDToPosition(double targetAngle){
        if (motor.getTargetPosition() != (int)targetAngle || motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            motor.setTargetPosition((int) targetAngle);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.7);
        }
    }

    public void toggleVision(){
        usingVision = !usingVision;
    }

    public void turnOnVision(){
        usingVision = true;
    }
    public void turnOffVision(){
        usingVision = false;
    }

    public void changeTargetPos(double v){
        targetPos += v;
    }

    // RoadRunner Localization 안될경우, 삭제 예정
    public Pose2d[] calculateMotionCompensation(Pose2d robotPose, Pose2d robotVel) {

        // 1. 실제 골대 위치 (상수에서 가져옴)
        Vector2d goalPos = new Vector2d(GOAL_X, GOAL_Y);

        // Field Centric 속도라고 가정 (vx, vy).
        Vector2d velocityVector = new Vector2d(robotVel.position.x, robotVel.position.y);

        // 3. 가상 타겟 계산 (목표 지점 += - v0 * dt)
        // 로봇이 움직이는 방향의 '반대'로 골대를 밀어버림
        Vector2d virtualGoalPos1 = goalPos.minus(velocityVector.times(SHOOTER_TIME_INTERVAL_ONE));
        Vector2d virtualGoalPos2 = goalPos.minus(velocityVector.times(SHOOTER_TIME_INTERVAL_ONE + SHOOTER_TIME_INTERVAL_TWO));

        // 4. 로봇에서 가상 골대까지의 벡터 계산
        Vector2d robotToVirtualGoal0 = goalPos.minus(robotPose.position);
        Vector2d robotToVirtualGoal1 = virtualGoalPos1.minus(robotPose.position);
        Vector2d robotToVirtualGoal2 = virtualGoalPos2.minus(robotPose.position);

        double targetAngleRad0 = normalizeAngle(
                Math.atan2(robotToVirtualGoal0.y, robotToVirtualGoal0.x) - robotPose.heading.log()
        );
        double targetAngleRad1 = normalizeAngle(
                Math.atan2(robotToVirtualGoal1.y, robotToVirtualGoal1.x) - robotPose.heading.log()
        );
        double targetAngleRad2 = normalizeAngle(
                Math.atan2(robotToVirtualGoal2.y, robotToVirtualGoal2.x) - robotPose.heading.log()
        );

        // 틱으로 변환 (Ticks per radian 등 상수 필요. 여기서는 예시)
        double targetAngleTick0 = angleToTicks(targetAngleRad0);
        double targetAngleTick1 = angleToTicks(targetAngleRad1);
        double targetAngleTick2 = angleToTicks(targetAngleRad2);

        // 슈터에게 넘겨줄 '가상 거리' 반환
        return new Pose2d[]{
                new Pose2d(robotToVirtualGoal0, targetAngleTick0),
                new Pose2d(robotToVirtualGoal1, targetAngleTick1),
                new Pose2d(robotToVirtualGoal2, targetAngleTick2)
        };
    }

    // 각도 정규화 헬퍼 함수
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // 라디안 -> 틱 변환 (Constants 값 사용 필요)
    private double angleToTicks(double radians) {
        // 예: 한 바퀴(2PI)가 TURRET_ONE_REV_TICKS
        return (radians / (2 * Math.PI)) * TURRET_ONE_REV_TICKS;
    }
}
