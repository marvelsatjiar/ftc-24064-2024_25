package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode.prototype;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;

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
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Intake intake = new Intake(hardwareMap);
        Extendo extendo = new Extendo(hardwareMap);

        Extendo.State targetPosition = Extendo.State.RETRACTED;

        waitForStart();

        while (opModeIsActive()) {
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(DPAD_UP)) intake.setTargetV4BAngle(Intake.V4BAngle.UP);
            if (gamepadEx1.wasJustPressed(DPAD_DOWN)) intake.setTargetV4BAngle(Intake.V4BAngle.DOWN);
            if (gamepadEx1.wasJustPressed(DPAD_RIGHT)) intake.setTargetV4BAngle(Intake.V4BAngle.CLEARING);
            if (gamepadEx1.wasJustPressed(DPAD_LEFT)) intake.setTargetV4BAngle(Intake.V4BAngle.UNSAFE);

            double trigger1 = gamepadEx1.getTrigger(RIGHT_TRIGGER) - gamepadEx1.getTrigger(LEFT_TRIGGER);

            intake.setRollerPower(trigger1);

            if (gamepadEx1.wasJustPressed(A)) targetPosition = Extendo.State.RETRACTED;
            if (gamepadEx1.wasJustPressed(B)) targetPosition = Extendo.State.EXTENDED;

            intake.run();

            extendo.setTargetAngle(targetPosition);

            extendo.run(intake.getTargetV4BAngle().isV4BUnsafe());

            extendo.printTelemetry();
            mTelemetry.update();
        }
    }
}
