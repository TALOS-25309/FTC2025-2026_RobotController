package org.firstinspires.ftc.teamcode.opmode.test; // 패키지명은 상황에 맞게 수정하세요

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.feature.TelemetrySystem;
import org.firstinspires.ftc.teamcode.feature.vision.Vision;
import org.firstinspires.ftc.teamcode.part.Constants;

import java.util.List;

@TeleOp(name = "Test: Vision Only", group = "Test")
public class VisionTest extends LinearOpMode {

    // Vision 객체 생성
    private final Vision vision = new Vision();


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        TelemetrySystem.init(telemetry);

        // 1. Bulk Read 설정 (Vision만 쓸 때는 필수까진 아니지만, 습관화하는 것이 좋음)
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // 2. Vision 초기화
        telemetry.addLine("Vision Initializing...");
        telemetry.update();


        vision.init(hardwareMap, telemetry);

        telemetry.addLine("Vision Initialized. Press Start.");
//        telemetry.addData("Limelight Status", vision.getStatus());
        telemetry.update();

        waitForStart();

        // 3. Vision 시작
        vision.start();
        vision.setPipeline(Constants.PIPELINE.RED_GOAL);

        // 루프 타임 측정을 위한 변수
        double lastTime = System.nanoTime();

        while (opModeIsActive()) {
            // --- 루프 속도 측정 시작 ---
            double currentTime = System.nanoTime();
            double loopTimeMs = (currentTime - lastTime) / 1e6; // 밀리초 단위 변환
            double loopHz = 1000.0 / loopTimeMs;                // 초당 루프 횟수(Hz)
            lastTime = currentTime;
            // ---------------------------

            // 4. Vision 업데이트 (여기서 Limelight 데이터를 받아옵니다)
            vision.update();

            // 5. 결과 출력
            telemetry.addData("=== Performance ===", "");
            telemetry.addData("Loop Rate (Hz)", "%.0f Hz", loopHz); // 목표: 50Hz 이상
            telemetry.addData("Loop Time (ms)", "%.1f ms", loopTimeMs);

            telemetry.addData("=== Vision Info ===", "");
            telemetry.addData("Time Stamp", vision.time);

            boolean detected = vision.tagDetected();
            telemetry.addData("Tag Detected?", detected);

            boolean longDeteced = vision.tagDetectedLong();
//            telemetry.addData("Tag Long Deteced Percentage", longDeteced);
            telemetry.addData("Tag Long Deteced?", longDeteced);

            if (detected) {
                // Vision.java에 getter가 있다면 아래처럼 값을 가져와 찍어봅니다.
                telemetry.addData("Fiducial ID", vision.getFiducialID());
                telemetry.addData("Distance", "%.2f", vision.getDistance());

                // Vision.java 내부적으로 TelemetrySystem을 통해 이미 Angle 등을 찍고 있을 수 있습니다.
            } else {
                telemetry.addLine("No Tag Visible");
            }

            // Vision.java 코드상 time이 -1이면 데이터가 없다는 뜻
            // (Vision.java 내부 로직 확인용)

            telemetry.update();
            TelemetrySystem.update();
        }
    }
}