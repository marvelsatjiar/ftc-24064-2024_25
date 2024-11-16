package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode.prototype;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Arm;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Claw;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Lift;

@Config
@TeleOp(name = "Arm Transfer Prototype",  group = "Prototype")
public final class ArmTransferPrototype extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        Lift lift = new Lift(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        Arm.ArmAngle targetArmAngle = Arm.ArmAngle.COLLECTING;
        Arm.WristAngle targetWristAngle = Arm.WristAngle.COLLECTING;
        Lift.Ticks targetTicks = Lift.Ticks.RETRACTED;

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            gamepadEx1.readButtons();

            boolean isAPressed = gamepadEx1.wasJustPressed(GamepadKeys.Button.A);
            boolean isBPressed = gamepadEx1.wasJustPressed(GamepadKeys.Button.B);
            boolean isDPADUpPressed = gamepadEx1.wasJustPressed(DPAD_UP);
            boolean isDPADDownPressed = gamepadEx1.wasJustPressed(DPAD_DOWN);

            if (gamepadEx1.wasJustPressed(Y)) claw.setClamped(!claw.getClamped());

            switch (targetArmAngle) {
                case COLLECTING:
                    if (isAPressed) targetArmAngle = Arm.ArmAngle.BASKET;
                    break;
                case BASKET:
                    if (isAPressed) targetArmAngle = Arm.ArmAngle.CHAMBER_FRONT;
                    break;
                case CHAMBER_FRONT:
                    if (isAPressed) targetArmAngle = Arm.ArmAngle.CHAMBER_BACK;
                    break;
                case CHAMBER_BACK:
                    if (isAPressed) targetArmAngle = Arm.ArmAngle.WALL_PICKUP;
                    break;
                case WALL_PICKUP:
                    if (isAPressed) targetArmAngle = Arm.ArmAngle.NEUTRAL;
                    break;
                case NEUTRAL:
                    if (isAPressed) targetArmAngle = Arm.ArmAngle.COLLECTING;
                    break;
            }

            switch (targetWristAngle) {
                case COLLECTING:
                    if (isBPressed) targetWristAngle = Arm.WristAngle.BASKET;
                    break;
                case BASKET:
                    if (isBPressed) targetWristAngle = Arm.WristAngle.CHAMBER_FRONT;
                    break;
                case CHAMBER_FRONT:
                    if (isBPressed) targetWristAngle = Arm.WristAngle.CHAMBER_BACK;
                    break;
                case CHAMBER_BACK:
                    if (isBPressed) targetWristAngle = Arm.WristAngle.WALL_PICKUP;
                    break;
                case WALL_PICKUP:
                    if (isBPressed) targetWristAngle = Arm.WristAngle.COLLECTING;
                    break;
            }

            switch (targetTicks) {
                case RETRACTED:
                    if (isDPADUpPressed) targetTicks = Lift.Ticks.LOW_BASKET;
                    break;
                case LOW_BASKET:
                    if (isDPADUpPressed) targetTicks = Lift.Ticks.HIGH_BASKET;
                    if (isDPADDownPressed) targetTicks = Lift.Ticks.RETRACTED;
                    break;
                case HIGH_BASKET:
                    if (isDPADUpPressed) targetTicks = Lift.Ticks.HIGH_CHAMBER_SETUP_BACK;
                    if (isDPADDownPressed) targetTicks = Lift.Ticks.LOW_BASKET;
                    break;
                case HIGH_CHAMBER_SETUP_BACK:
                    if (isDPADUpPressed) targetTicks = Lift.Ticks.HIGH_CHAMBER_SCORE_BACK;
                    if (isDPADDownPressed) targetTicks = Lift.Ticks.HIGH_BASKET;
                    break;
                case HIGH_CHAMBER_SCORE_BACK:
                    if (isDPADUpPressed) targetTicks = Lift.Ticks.HIGH_CHAMBER_SETUP_FRONT;
                    if (isDPADDownPressed) targetTicks = Lift.Ticks.HIGH_CHAMBER_SETUP_BACK;
                    break;
                case HIGH_CHAMBER_SETUP_FRONT:
                    if (isDPADUpPressed) targetTicks = Lift.Ticks.HIGH_CHAMBER_SCORE_FRONT;
                    if (isDPADDownPressed) targetTicks = Lift.Ticks.HIGH_CHAMBER_SCORE_BACK;
                    break;
                case HIGH_CHAMBER_SCORE_FRONT:
                    if (isDPADUpPressed) targetTicks = Lift.Ticks.WALL_PICKUP;
                    if (isDPADDownPressed) targetTicks = Lift.Ticks.HIGH_CHAMBER_SETUP_FRONT;
                    break;
                case WALL_PICKUP:
                    if (isDPADUpPressed) targetTicks = Lift.Ticks.CLIMB;
                    if (isDPADDownPressed) targetTicks = Lift.Ticks.HIGH_CHAMBER_SCORE_FRONT;
                    break;
                case CLIMB:
                    if (isDPADUpPressed) targetTicks = Lift.Ticks.EXTENDED;
                    if (isDPADDownPressed) targetTicks = Lift.Ticks.WALL_PICKUP;
                    break;
                case EXTENDED:
                    if (isDPADDownPressed) targetTicks = Lift.Ticks.CLIMB;
                    break;
            }

            arm.setArmAngle(targetArmAngle);
            arm.setWristAngle(targetWristAngle);
            lift.setTargetTicks(targetTicks);
            claw.run();

            arm.run(lift.getTargetTicks().isArmUnsafe());
            lift.run();

            lift.printTelemetry();
            arm.printTelemetry();
            mTelemetry.update();
        }
    }
}
