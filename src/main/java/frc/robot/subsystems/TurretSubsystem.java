package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

    // ---------------- Flywheel Constants ----------------
    public static final double MAX_RPM            = 6000.0;
    private static final double MIN_SHOOT_RPM     = 2200.0;
    private static final double MIN_DISTANCE_METERS = 1.0;
    private static final double MAX_DISTANCE_METERS = 6.0;
    private static final int    FLYWHEEL_CAN_ID   = 21;

    // ---------------- flywheelspin Constants ----------------
    private static final int    flywheelspin_CAN_ID          =  11;
    private static final double flywheelspin_SPEED_FORWARD   =  0.2;
    private static final double flywheelspin_SPEED_REVERSE   = -0.2;

    // ---------------- Hardware ----------------
    private final TalonFX        flywheelMotor    = new TalonFX(FLYWHEEL_CAN_ID);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final DutyCycleOut   dutyCycleRequest = new DutyCycleOut(0);
    private final SparkMax       flywheelspinMotor = new SparkMax(flywheelspin_CAN_ID, MotorType.kBrushless);

    // ---------------- State ----------------
    private boolean isRunning = false;

    // ------------------------------------------------
    public TurretSubsystem() {
        configureMotor();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.CurrentLimits.SupplyCurrentLimit       = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit       = 80;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12;

        flywheelMotor.getConfigurator().apply(config);
    }

    // ---------------- Flywheel Methods ----------------

    public void setTargetRPM(double rpm) {
        double clampedRPM = MathUtil.clamp(rpm, 0.0, MAX_RPM);
        flywheelMotor.setControl(velocityRequest.withVelocity(clampedRPM / 60.0));
    }

    public void setManualOutput(double percentOutput) {
        double clampedOutput = MathUtil.clamp(percentOutput, -1.0, 1.0);
        flywheelMotor.setControl(dutyCycleRequest.withOutput(clampedOutput));
    }

    public void stop() {
        flywheelMotor.stopMotor();
    }

    public double getCurrentRPM() {
        return flywheelMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    public double getRPMForDistance(double distanceMeters) {
        double clampedDistance = MathUtil.clamp(distanceMeters, MIN_DISTANCE_METERS, MAX_DISTANCE_METERS);
        double normalized = (clampedDistance - MIN_DISTANCE_METERS)
            / (MAX_DISTANCE_METERS - MIN_DISTANCE_METERS);
        return MIN_SHOOT_RPM + normalized * (MAX_RPM - MIN_SHOOT_RPM);
    }

    // ---------------- flywheelspin Methods ----------------

    public void turnRight() {
        isRunning = true;
        flywheelspinMotor.set(flywheelspin_SPEED_FORWARD);
    }

    public void turnLeft() {
        isRunning = true;
        flywheelspinMotor.set(flywheelspin_SPEED_REVERSE);
    }

    public void stopSpinning() {
        isRunning = false;
        flywheelspinMotor.set(0);
    }

    public boolean isRunning() {
        return isRunning;
    }

    public double getflywheelspinCurrent() {
        return flywheelspinMotor.getOutputCurrent();
    }

    // ---------------- Single Merged Periodic ----------------
    @Override
    public void periodic() {
        // Flywheel telemetry
        SmartDashboard.putNumber("Shooter/RPM",     getCurrentRPM());
        SmartDashboard.putNumber("Shooter/Voltage", flywheelMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Current", flywheelMotor.getStatorCurrent().getValueAsDouble());

        // flywheelspin telemetry
        SmartDashboard.putBoolean("flywheelspin/IsRunning", isRunning);
        SmartDashboard.putNumber("flywheelspin/Current",    flywheelspinMotor.getOutputCurrent());
        SmartDashboard.putNumber("flywheelspin/Voltage",    flywheelspinMotor.getBusVoltage());
        SmartDashboard.putNumber("flywheelspin/Speed",      flywheelspinMotor.getEncoder().getVelocity());
    }
}
