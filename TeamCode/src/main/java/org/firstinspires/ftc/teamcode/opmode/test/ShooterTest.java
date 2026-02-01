package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.feature.TelemetrySystem;
import org.firstinspires.ftc.teamcode.feature.vision.Vision;
import org.firstinspires.ftc.teamcode.part.intake.Intake;
import org.firstinspires.ftc.teamcode.part.shooter.Shooter;
import org.firstinspires.ftc.teamcode.part.Turret;
import org.firstinspires.ftc.teamcode.part.Constants;

// 1. 여기에 이름을 지정하면 Dashboard에 'ShooterTuning'이라는 그룹이 새로 생깁니다.
@Config("ShooterTest_0705")
@TeleOp(name = "ShooterTest_0705", group = "Tuning")
public class ShooterTest extends LinearOpMode {

    // =========================================================
    // Dashboard에서 조종할 변수들 (public static 필수)
    // =========================================================

    // 모드 설정
    public static boolean ENABLE_SHOOTER = false;    // 체크하면 모터 회전 시작
    public static boolean ENABLE_INTAKE = false;    // 체크하면 모터 회전 시작
    public static boolean OPEN_STOPPER = false;      // 체크하면 링 발사 (스토퍼 열림)
    public static boolean USE_AUTO_CALCULATION = false; // 체크하면 거리기반 자동 각도 검증, 해제하면 수동 튜닝
    public static boolean ENABLE_TURRET_TRACKING = true; // 터렛 자동 추적 여부
    public static boolean RED = true;

    // 수동 튜닝 값 (USE_AUTO_CALCULATION이 꺼져있을 때 사용)
    public static double TARGET_VELOCITY = 5.0;      // 목표 속도 (m/s)
    public static double TARGET_HOOD_ANGLE = 0.5;    // 목표 서보 각도 (0.0 ~ 1.0)

    // =========================================================

    private Shooter shooter;
    private Turret turret;
    private Vision vision;
    private Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        TelemetrySystem.init(telemetry);

        vision = new Vision();
        shooter = new Shooter();
        turret = new Turret(vision);
        intake = new Intake();

        vision.init(hardwareMap, telemetry);
        shooter.init(hardwareMap, telemetry);
        turret.init(hardwareMap, telemetry);
        intake.init(hardwareMap, telemetry);

        if (RED)  vision.setPipeline(Constants.PIPELINE.RED_GOAL);
        else vision.setPipeline(Constants.PIPELINE.BLUE_GOAL);

        telemetry.addLine("Dashboard > 'ShooterTuning' 그룹을 열어서 제어하세요.");
        telemetry.update();

        waitForStart();

        vision.start();
        shooter.start();
        turret.start();
        intake.start();

        while (opModeIsActive()) {
            vision.update();
            double currentDistance = vision.getDistance();

            telemetry.addData("Vision Detected",vision.tagDetected());

            // -----------------------------------------------------
            // 1. 터렛 제어
            // -----------------------------------------------------













            if (ENABLE_TURRET_TRACKING) {
                turret.runPIDWithVision();
            } else {
                turret.stop();
            }

            // -----------------------------------------------------
            // 2. 슈터 각도 및 속도 결정
            // -----------------------------------------------------
            double appliedAngle;
            double appliedVelocity = TARGET_VELOCITY; // 속도는 일단 수동 설정값 따름

            if (USE_AUTO_CALCULATION) {
                // 자동 모드: 현재 거리에 따른 보간 값 적용 (검증용)
                appliedAngle = shooter.getInterpolatedAngle(currentDistance);
                telemetry.addData("[Mode]", "Auto Calculation");
            } else {
                // 수동 모드: 대시보드 입력값 적용 (튜닝용)
                appliedAngle = TARGET_HOOD_ANGLE;
                telemetry.addData("[Mode]", "Manual Tuning");
            }

            // 하드웨어에 값 적용
            shooter.setAngle(appliedAngle);

            // -----------------------------------------------------
            // 3. 슈터 모터 ON/OFF
            // -----------------------------------------------------
            if (ENABLE_SHOOTER) {
                // Shooter 클래스 필드 직접 제어 (속도 적용)
                shooter.shooterMotorUpper.setVelocity(appliedVelocity, AngleUnit.RADIANS);
                shooter.shooterMotorLower.setVelocity(appliedVelocity, AngleUnit.RADIANS);
            } else {
                shooter.cmdShooterStop();
            }

            if (ENABLE_INTAKE){
                intake.cmdRun();
            }
            else{
                intake.cmdStop();
            }


            // -----------------------------------------------------
            // 4. 발사 (스토퍼) 제어
            // -----------------------------------------------------
            if (OPEN_STOPPER) {
                shooter.cmdStopperOpen();
            } else {
                shooter.cmdStopperClose();
            }

            // -----------------------------------------------------
            // Telemetry: 튜닝에 필요한 핵심 정보 표시
            // -----------------------------------------------------
            telemetry.addData("Vision Distance", "%.3f m", currentDistance);
            telemetry.addData("Applied Hood Angle", "%.4f", appliedAngle);
            telemetry.addData("Applied Velocity", "%.2f", appliedVelocity);

            // 실제 모터가 목표 속도를 잘 따라가는지 확인
            double actualVel = shooter.shooterMotorUpper.getVelocity(AngleUnit.RADIANS);
            telemetry.addData("Real Velocity", "%.2f (Err: %.2f)", actualVel, appliedVelocity - actualVel);

            shooter.update();
            turret.update();
            telemetry.update();
        }

        vision.stop();
        shooter.stop();
        turret.stop();
    }
}