package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.BulkReader;

@TeleOp(name = "Intake Prototype", group = "Mechanism Test")
public class IntakePrototype extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        MotorEx intake1 = new MotorEx(hardwareMap, "intake1");
        MotorEx intake2 = new MotorEx(hardwareMap, "intake2");

        BulkReader bulkReader = new BulkReader(hardwareMap);

        MotorEx[] motorExes = {intake1, intake2};

        intake2.setInverted(true);

        waitForStart();

        while (opModeIsActive()) {
            bulkReader.bulkRead();
            double trigger1 = gamepadEx1.getTrigger(RIGHT_TRIGGER) - gamepadEx1.getTrigger(LEFT_TRIGGER);

            for (MotorEx motorEx: motorExes) {
                motorEx.set(trigger1);
            }
        }

    }
}
