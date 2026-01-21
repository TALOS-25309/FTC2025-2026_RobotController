package org.firstinspires.ftc.teamcode.part.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.feature.TelemetrySystem;
import org.firstinspires.ftc.teamcode.part.Constants;
import org.firstinspires.ftc.teamcode.part.Part;
import org.firstinspires.ftc.teamcode.part.vision.Vision;



public class Shooter implements Part {
    DcMotorEx shooterMotorUpper;
    DcMotorEx shooterMotorLower;

    Servo shooterServo;
    Servo stopper;

    Vision vision;

    public StopperState stopperState;
    public ShooterState shooterState;
    public double hoodAngle;

    public Shooter(Vision vision){
        this.vision = vision;
    }

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        shooterMotorUpper = hardwareMap.get(DcMotorEx.class, "shooter_motor1");
        shooterMotorLower = hardwareMap.get(DcMotorEx.class, "shooter_motor2");

        shooterServo = hardwareMap.get(Servo.class, "servo_hood");
        stopper = hardwareMap.get(Servo.class, "servo_stopper");
    }

    @Override
    public void start() {
        shooterMotorUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        cmdShooterRun();

        shooterServo.setPosition(Constants.SHOOTER_DOWN_ANGLE);
        stopper.setPosition(Constants.STOPPER_CLOSE_ANGLE);

        shooterState = ShooterState.RUN;
        stopperState = StopperState.CLOSE;

    }

    @Override
    public void update() {
        TelemetrySystem.addClassData("Shooter", "ShooterRunning", Constants.SHOOTER_RUNNING);

        if (!Constants.SHOOTER_RUNNING){
            cmdShooterStop();
        }
    }

    @Override
    public void stop() {
        cmdShooterStop();
    }


    public void stopperOpen(){
        stopper.setPosition(Constants.STOPPER_OPEN_ANGLE);
        stopperState = StopperState.OPEN;
    }
    public void stopperClose(){
        stopper.setPosition(Constants.STOPPER_CLOSE_ANGLE);
        stopperState = StopperState.CLOSE;
    }



    // Commands
    public void cmdShooterRun(){
        shooterMotorUpper.setPower(Constants.SHOOTER_POWER);
        shooterMotorLower.setPower(-1 * Constants.SHOOTER_POWER);
        shooterState = ShooterState.RUN;
    }
    public void cmdShooterStop(){
        shooterMotorUpper.setPower(0);
        shooterMotorLower.setPower(0);
        shooterState = ShooterState.STOP;
    }

    public void cmdSetServoAngle(double angle){
        shooterServo.setPosition(angle);
    }

    public void cmdStopperToggle() {
        if (this.stopperState == StopperState.OPEN) {
            this.stopperClose();
        } else if (this.stopperState == StopperState.CLOSE) {
            this.stopperOpen();
        } else {
            this.stopperClose();
        }
    }

    //버릴코드!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //판별식 too risky
    //각도랑 대응하는 거리 모아두고 이분탐색으로 각도 계산
    public double calculateShooterAngle(double V, double d, double e1, double h) {
        double g = 9.81; // 중력가속도
        double X = d + e1; // 수평 거리 합

        // 2차 방정식 계수 계산 (A*tan^2 - B*tan + C = 0 형태에서 변형)
        // 수식: (g*X^2 / 2*V^2) * tan^2 - X * tan + (h + g*X^2 / 2*V^2) = 0

        double commonFactor = (g * X * X) / (2 * V * V);

        double A = commonFactor;
        double B = -X;
        double C = h + commonFactor;

        // 판별식 (Discriminant)
        double delta = B * B - 4 * A * C;

        if (delta < 0) {
            // [물리적 불가능] 현재 속도(V)로는 거리가 너무 멀거나 높아서 닿지 않음
            return Double.NaN;
        }

        // 근의 공식 적용 (일반적으로 낮은 궤적을 위해 - 부호 사용)
        // 만약 높은 궤적(박격포 샷)이 필요하면 Math.sqrt(delta) 앞을 +로 변경
        double tanTheta = (-B - Math.sqrt(delta)) / (2 * A);

        // 라디안을 도(Degree)로 변환
        return Math.toDegrees(Math.atan(tanTheta));
    }
}
