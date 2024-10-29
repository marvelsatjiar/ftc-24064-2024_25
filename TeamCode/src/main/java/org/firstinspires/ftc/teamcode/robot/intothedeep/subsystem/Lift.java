package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_435;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.MAX_VOLTAGE;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.filter.singlefilter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.FeedforwardGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.LowPassGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;

@Config
public final class Lift {

    /**
     * A PIDGains object being set to certain values (tweak these numbers!!)
     */
    public static PIDGains pidGains = new PIDGains(
            0.005,
            0.002,
            0.0001,
            Double.POSITIVE_INFINITY
    );

    public static FeedforwardGains feedforwardGains = new FeedforwardGains(
            0.01,
            0.01
    );

    // Sets the filter for PID outputs and constrains overshoots with controlling (also tweak!!)
    public static LowPassGains filterGains = new LowPassGains(0, 2);

    /**
     * Sets the constants for the positions, conversions, etc
     * Remember to set these constants correctly! (in ticks)
     */
    public static int
            MAX_MOTOR_TICKS = 2350,
            MIN_MOTOR_TICKS = -5,
            BASKET_TICKS = 800,
            CHAMBER1_TICKS = 600,
            CHAMBER2_TICKS = 1000,
            CLIMB_TICKS = 1200,
            UNSAFE_THRESHOLD_TICKS = 50;
    public static double
            kG = 0.011065,
            JOYSTICK_MULTIPLIER = 40; // 1 = 40 ticks

    private final MotorEx[] motors;

    private final Motor.Encoder encoder;

    private final FIRLowPassFilter derivFilter = new FIRLowPassFilter(filterGains);
    private final PIDController controller = new PIDController(derivFilter);

    private final VoltageSensor batteryVoltageSensor;

    private State currentState = new State();

    public enum SlideTicks {
        RETRACTED,
        BASKET,
        CHAMBER1,
        CHAMBER2,
        CLIMB,
        EXTENDED;

        private int getTicks() {
            switch (this) {
                case BASKET:
                    return BASKET_TICKS;
                case CHAMBER1:
                    return CHAMBER1_TICKS;
                case CHAMBER2:
                    return CHAMBER2_TICKS;
                case CLIMB:
                    return CLIMB_TICKS;
                case EXTENDED:
                    return MAX_MOTOR_TICKS;
                default:
                    return MIN_MOTOR_TICKS;
            }
        }

        public boolean isArmUnsafe() {
            return getTicks() <= UNSAFE_THRESHOLD_TICKS;
        }
    }

    private SlideTicks setPoint = SlideTicks.RETRACTED;

    /**
     * Constructor of Lift class; Sets variables with hw (hardwareMap)
     * @param hardwareMap; A constant map that holds all the parts for config in code
     */
    public Lift(HardwareMap hardwareMap) {
        MotorEx leader = new MotorEx(hardwareMap, "leader", RPM_435);
        MotorEx follower = new MotorEx(hardwareMap, "follower", RPM_435);

        encoder = new MotorEx(hardwareMap, "right back", RPM_435).encoder;
        encoder.reset();

        follower.setInverted(true);

        motors = new MotorEx[] {leader, follower};

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public SlideTicks getSetPoint() {
        return setPoint;
    }

    public void setPosition(SlideTicks slideTicks) {
        setPoint = slideTicks;
        controller.setTarget(new State(getSetPoint().getTicks()));
    }

    /**
     * Sets the three variables; The currentState sets to the position of the motor; Other two set the constants of the gains
     * Calls another run() method that calculates the motor output proportionally and doesn't compensate for power
     */
    public void run() {
        currentState = new State(encoder.getPosition());
        controller.setGains(pidGains);
        derivFilter.setGains(filterGains);

        run(controller.calculate(currentState), false);
    }

    public void run(double motorPower) {
        run(motorPower, true);
    }

    /**
     * Checks if the voltage is to be saved, if true, use scalar to modify motor power
     * @param motorPower; The power that is set to be used to set the motor's power
     * @param voltageCompensate; Boolean that is used if battery is low, and if it needs to compensate (save)
     */
    public void run(double motorPower, boolean voltageCompensate) {
        double scalar = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();

        if (voltageCompensate) motorPower *= scalar;

        for (MotorEx motor : motors) motor.set(motorPower + kG * scalar);
    }

    public void printTelemetry() {
        mTelemetry.addData("Target position (ticks)", getSetPoint().getTicks());
        mTelemetry.addData("Current state (name)", getSetPoint().name());
    }

    public void printNumericalTelemetry() {
        mTelemetry.addData("Current position (ticks)", currentState.x);
        mTelemetry.addData("Error derivative (ticks/s)", controller.getFilteredErrorDerivative());
        mTelemetry.addData("Error (ticks)", controller.getErrorIntegral());
        mTelemetry.addData("kD (computed)", pidGains.kD);
    }
}
