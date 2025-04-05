package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public final class Hang {
    private final CRServo[] hangGroup;

    private double input;

    public Hang(HardwareMap hardwareMap) {
        CRServo hangMaster = hardwareMap.get(CRServo.class, "hang master");
        CRServo hangFollower = hardwareMap.get(CRServo.class, "hang follower");

        hangFollower.setDirection(DcMotorSimple.Direction.REVERSE);

        hangGroup = new CRServo[] {hangMaster, hangFollower};
    }

    public void setPower(double power) {
        input = power;
    }

    public void run() {
        for (CRServo servo : hangGroup) {
            servo.setPower(input);
        }
    }

}
