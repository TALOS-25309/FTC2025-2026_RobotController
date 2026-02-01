package org.firstinspires.ftc.teamcode.feature.vision;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.code.Type;

import static org.firstinspires.ftc.teamcode.part.Constants.*;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.feature.TelemetrySystem;

import java.util.List;

class StatusData {
    boolean status;
    double time;

    public StatusData(boolean status, double time) {
        this.status = status;
        this.time = time;
    }
}

@Config(value = "Vision")
public class Vision {
    private  Limelight3A LL;

    private double angle, distance;
    private boolean detected;

    private StatusData[] detectionCache;
    private int pointer;
    private final int MEMORY_SIZE = 100;
    public static int CACHE_TIME_LIMIT = 1000;
    private final double DETECTION_SUCCESS_PERCENT = 0.5;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        LL = hardwareMap.get(Limelight3A.class, "Limelight");
    }
    public void start() {;
        LL.setPollRateHz(40);
        LL.start();

        detected = false;
        distance = 0;


        detectionCache = new StatusData[MEMORY_SIZE];
        for (int i = 0; i < MEMORY_SIZE; i++) {
            detectionCache[i] = new StatusData(false, -5000);
        }
        pointer = 1;
    }


    public void setPipeline(PIPELINE pipeline) {
        switch (pipeline){
            case RED_GOAL:
                LL.pipelineSwitch(RED_PIPELINE);
                break;
            case BLUE_GOAL:
                LL.pipelineSwitch(BLUE_PIPELINE);
                break;
            case OBELISK_DETECTION:
                LL.pipelineSwitch(OBELISK_PIPELINE);
                break;
            default:
                LL.pipelineSwitch(RED_PIPELINE);
                break;
        }
    }

    public List<FiducialResult> fiducials;
//    public LLStatus status;
    public double time;


    public void update(){

        LLResult result = LL.getLatestResult();
//        status = LL.getStatus();
        time = result.getTimestamp(); // Limelight 내부 시간 사용

        if (result != null && result.isValid()) {

            fiducials = result.getFiducialResults();
            if (fiducials.isEmpty()){
                detected = false;
            }
            else {
                detected = true;
                FiducialResult fiduciary = fiducials.get(0);

                angle = result.getTx();

                distance = Math.pow(
                            Math.pow(fiduciary.getTargetPoseCameraSpace().getPosition().z, 2)
                                + Math.pow(fiduciary.getTargetPoseCameraSpace().getPosition().x, 2),
                            0.5
                        );
                TelemetrySystem.addClassData("Vision","x",fiduciary.getTargetPoseRobotSpace().getPosition().x);
                TelemetrySystem.addClassData("Vision","z",fiduciary.getTargetPoseRobotSpace().getPosition().z);


            }

        }
        else{
            detected = false;
//            time = -1;
        }

        if (time > 0 && detectionCache[(pointer-1+MEMORY_SIZE)%MEMORY_SIZE].time < time - 1){
            detectionCache[pointer] = new StatusData(detected, time);
            pointer = (pointer + 1)%MEMORY_SIZE;
        }


        TelemetrySystem.addClassData("Vision","Timestamp", time);
        TelemetrySystem.addClassData("Vision","distance", distance);
    }

    public int getFiducialID(){
        return fiducials.get(0).getFiducialId();
    }

    // Warning : getting the status of Limelight takes a lot of time.
    // To provide enough working rate, remove this method in the main loop.
    public LLStatus getStatus(){
        return LL.getStatus();
    }

    public boolean tagDetected(){
        return detected;
    }

//    public double tagDetectedLong(){
    public boolean tagDetectedLong(){
        int count = 0;
        int num_all = 0;
        for (int i = 0; i < MEMORY_SIZE; i++) {
            StatusData data = detectionCache[i];
                if (data.time > 0 && (time - data.time) < CACHE_TIME_LIMIT) {
                num_all++;
                if (data.status) count++;
            }
        }
//        if (num_all == 0) return false;
//        return (double) count / num_all > DETECTION_SUCCESS_PERCENT;
        TelemetrySystem.addClassData("vision","num all", num_all);
        TelemetrySystem.addClassData("vision","count", count);
        if (num_all == 0) return false;
        TelemetrySystem.addClassData("vision", "percent", count/num_all);

        return ((double) count / num_all) > DETECTION_SUCCESS_PERCENT;
    }

    public double getDistance(){
        return distance;
    }

    public double getAngle(){
        return angle;
    }



    public void stop(){
        LL.stop();
    }


}
