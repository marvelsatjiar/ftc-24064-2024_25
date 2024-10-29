package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode.prototype;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Extendo;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;

@Config
@TeleOp(name = "Intake Prototype",  group = "Prototype")
public final class IntakePrototype extends LinearOpMode {
    Intake intake;
    Extendo extendo;
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new Intake(hardwareMap);
        extendo = new Extendo(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(DPAD_UP)) intake.setTarget(Intake.V4BAngles.UP);
            if (gamepadEx1.wasJustPressed(DPAD_DOWN)) intake.setTarget(Intake.V4BAngles.DOWN);
            if (gamepadEx1.wasJustPressed(DPAD_RIGHT)) intake.setTarget(Intake.V4BAngles.CLEARING);
            if (gamepadEx1.wasJustPressed(DPAD_LEFT)) intake.setTarget(Intake.V4BAngles.UNSAFE);

            double trigger1 = gamepadEx1.getTrigger(RIGHT_TRIGGER) - gamepadEx1.getTrigger(LEFT_TRIGGER);

            intake.setServoPower(trigger1);

            double leftStick = gamepadEx1.getLeftY();
            if (leftStick != 0) extendo.setWithStick(leftStick);

            intake.run();
            extendo.run(intake.getTargetAngle().isV4BUnsafe());

            extendo.printTelemetry();
            mTelemetry.update();
        }
    }
}
