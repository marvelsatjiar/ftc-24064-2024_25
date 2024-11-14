package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode.prototype;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Arm;

@Config
@TeleOp(name = "Arm Prototype",  group = "Prototype")
public final class ArmPrototype extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        Arm arm = new Arm(hardwareMap);

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Arm.ArmAngle targetArmAngle = Arm.ArmAngle.COLLECTING;

        waitForStart();

        while (opModeIsActive()) {
            gamepadEx1.readButtons();

            boolean isAPressed = gamepadEx1.wasJustPressed(GamepadKeys.Button.A);

            switch (targetArmAngle) {
                case COLLECTING:
                    if (isAPressed) targetArmAngle = Arm.ArmAngle.BASKET;
                    break;
                case BASKET:
                    if (isAPressed) targetArmAngle = Arm.ArmAngle.CHAMBER_FRONT;
                    break;
                case CHAMBER_FRONT:
                    if (isAPressed) targetArmAngle = Arm.ArmAngle.NEUTRAL;
                    break;
                case NEUTRAL:
                    if (isAPressed) targetArmAngle = Arm.ArmAngle.COLLECTING;
                    break;
            }

            arm.setArmAngle(targetArmAngle);

            arm.run(false);

            arm.printTelemetry();
            mTelemetry.update();
        }
    }
}
