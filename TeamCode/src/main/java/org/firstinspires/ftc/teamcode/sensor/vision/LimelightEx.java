package org.firstinspires.ftc.teamcode.sensor.vision;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.net.InetAddress;
import java.util.List;

public final class LimelightEx {
    LLResult result;
    private final Limelight3A limelight;

    public LimelightEx(Limelight3A limelight) {
        this.limelight = limelight;
    }

    public void update() {
        result = limelight.getLatestResult();
    }

    public List<LLResultTypes.ColorResult> getColorResult() {
        return result.getColorResults();
    }

    public List<LLResultTypes.DetectorResult> getDetectorResult() {
        if (result != null) return result.getDetectorResults();
        return null;
    }

    public Pose2d getPoseEstimate() {
        Pose2d pose = null;
        if (result != null && result.isValid()) {
            Pose3D pose3D = result.getBotpose_MT2();
            if (pose3D != null) {
                pose = new Pose2d(pose3D.getPosition().x, pose3D.getPosition().y, 0);
            }
        }
        return pose;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }
}