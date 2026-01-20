package org.firstinspires.ftc.teamcode.part;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter implements Part {
    DcMotorEx shooterMotorUpper;
    DcMotorEx shooterMotorLower;

    Servo shooterServo;
    Servo stopper;

    Vision vision;

    public Shooter(Vision vision){
        this.vision = vision;
    }

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        shooterMotorUpper = hardwareMap.get(DcMotorEx.class, "shooterMotorUpper");
        shooterMotorLower = hardwareMap.get(DcMotorEx.class, "shooterMotorLower");

        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        stopper = hardwareMap.get(Servo.class, "stopper");
    }

    @Override
    public void start() {
        shooterMotorUpper.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotorUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorLower.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotorLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {

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
