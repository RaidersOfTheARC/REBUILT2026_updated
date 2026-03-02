package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    // ---------------- Intake Constants ----------------
    private static final int    INTAKE_CAN_ID        =  9;
    private static final double INTAKE_SPEED_FORWARD =  0.6;
    private static final double INTAKE_SPEED_REVERSE = -0.6;

    // ---------------- Intake Lift Constants ----------------
    private static final int    INTAKELIFT_CAN_ID        = 10;
    private static final double INTAKELIFT_SPEED_FORWARD =  0.3;
    private static final double INTAKELIFT_SPEED_REVERSE = -0.1;

    // -------------------------------------------------------
    // GEAR RATIO — change this to match your physical setup.
    // Example: if your lift gearbox is 20:1, set this to 20.0
    // This converts motor rotations to arm rotations.
    // -------------------------------------------------------
    private static final double LIFT_GEAR_RATIO = 9.0;

    // 90 degrees = 0.25 arm rotations * gear ratio = motor rotations
    private static final double LIFT_90_DEG_ROTATIONS = (110.0 / 360.0) * LIFT_GEAR_RATIO;

    // PID gains for position hold — tune these on the robot
    private static final double LIFT_kP = 0.08;  // increase if arm is slow to reach target
    private static final double LIFT_kI = 0.0;  // leave at 0 until P is tuned
    private static final double LIFT_kD = 0.1;  // increase if arm overshoots

    // ---------------- Hardware ----------------
    private final SparkMax intakeMotor     = new SparkMax(INTAKE_CAN_ID,     MotorType.kBrushless);
    private final SparkMax intakeLiftMotor = new SparkMax(INTAKELIFT_CAN_ID, MotorType.kBrushless);

    // Lift position control
    private final SparkClosedLoopController liftPID     = intakeLiftMotor.getClosedLoopController();
    private final RelativeEncoder           liftEncoder = intakeLiftMotor.getEncoder();

    // ---------------- State ----------------
    private boolean isIntakeRunning  = false;
    private boolean isLiftRunning    = false;
    private boolean isHolding90      = false;

    // ------------------------------------------------
    public IntakeSubsystem() {
        configureIntakeLift();
    }

    private void configureIntakeLift() {
    SparkMaxConfig config = new SparkMaxConfig();

    // Primary encoder is default feedback sensor — no need to declare it
    config.closedLoop
        .p(LIFT_kP)
        .i(LIFT_kI)
        .d(LIFT_kD)
        .outputRange(-0.3, 0.3);

    liftEncoder.setPosition(0);

    intakeLiftMotor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
}

    // ---------------- Intake Roller Methods ----------------

    public void turnRight() {
        isIntakeRunning = true;
        intakeMotor.set(INTAKE_SPEED_FORWARD);
    }

    public void turnLeft() {
        isIntakeRunning = true;
        intakeMotor.set(INTAKE_SPEED_REVERSE);
    }

    public void stop() {
        isIntakeRunning = false;
        intakeMotor.set(0);
    }

    // ---------------- Intake Lift Manual Methods ----------------

    public void raiseUp() {
        isHolding90  = false;
        isLiftRunning = true;
        intakeLiftMotor.set(INTAKELIFT_SPEED_FORWARD);
    }

    public void raiseDown() {
        isHolding90  = false;
        isLiftRunning = true;
        intakeLiftMotor.set(INTAKELIFT_SPEED_REVERSE);
    }

    public void stopLifting() {
        // Only stop if we are NOT in a position hold
        // If dpad left is active we want the PID to keep holding
        if (!isHolding90) {
            isLiftRunning = false;
            intakeLiftMotor.set(0);
        }
    }

    // ---------------- Intake Lift Position Methods ----------------

    /**
     * Commands the lift to 90 degrees and holds using onboard PID.
     * Called when dpad left is pressed.
     */
    public void holdAt90() {
        isHolding90   = true;
        isLiftRunning = true;
        liftPID.setReference(LIFT_90_DEG_ROTATIONS, ControlType.kPosition);
    }

    /**
     * Releases the position hold.
     * Called when dpad left is released.
     */
    public void releaseHold() {
        isHolding90   = false;
        isLiftRunning = false;
        intakeLiftMotor.set(0);
    }

    /**
     * Resets encoder to zero — call this when arm is physically at rest position.
     */
    public void resetEncoder() {
        liftEncoder.setPosition(0);
    }

    // ---------------- Getters ----------------

    public boolean isIntakeRunning() { return isIntakeRunning; }
    public boolean isLiftRunning()   { return isLiftRunning;   }
    public boolean isHolding90()     { return isHolding90;     }

    public double getLiftPosition()  { return liftEncoder.getPosition(); }
    public double getIntakeCurrent() { return intakeMotor.getOutputCurrent();     }
    public double getLiftCurrent()   { return intakeLiftMotor.getOutputCurrent(); }

    // ---------------- Telemetry ----------------
    @Override
    public void periodic() {
        // Intake roller
        SmartDashboard.putBoolean("Intake/IsRunning", isIntakeRunning);
        SmartDashboard.putNumber("Intake/Current",    intakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("Intake/Voltage",    intakeMotor.getBusVoltage());
        SmartDashboard.putNumber("Intake/Speed",      intakeMotor.getEncoder().getVelocity());

        // Intake lift
        SmartDashboard.putBoolean("IntakeLift/IsRunning",  isLiftRunning);
        SmartDashboard.putBoolean("IntakeLift/Holding90",  isHolding90);
        SmartDashboard.putNumber("IntakeLift/PositionRot", liftEncoder.getPosition());
        SmartDashboard.putNumber("IntakeLift/Target90Rot", LIFT_90_DEG_ROTATIONS);
        SmartDashboard.putNumber("IntakeLift/Current",     intakeLiftMotor.getOutputCurrent());
        SmartDashboard.putNumber("IntakeLift/Voltage",     intakeLiftMotor.getBusVoltage());
    }
}