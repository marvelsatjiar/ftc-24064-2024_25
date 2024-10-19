package org.firstinspires.ftc.teamcode.sensor.vision;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.net.InetAddress;
import java.util.List;

public class LimelightEx extends Limelight3A {
    LLResult result;

    public LimelightEx(SerialNumber serialNumber, String name, InetAddress ipAddress) {
        super(serialNumber, name, ipAddress);
    }

    public void update() {
        result = this.getLatestResult();
    }

    public List<LLResultTypes.ColorResult> getColorResult() {
        return result.getColorResults();
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
}