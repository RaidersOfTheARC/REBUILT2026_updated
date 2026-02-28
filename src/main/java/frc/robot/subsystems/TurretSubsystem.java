package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

    // ---------------- CAN ID ----------------
    private static final int FLYWHEEL_CAN_ID = 21;

    // ---------------- Hardware ----------------
    private final TalonFX flywheelMotor = new TalonFX(FLYWHEEL_CAN_ID);

    // Velocity control request
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    // ---------------- Distance → RPM Map ----------------
    private final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();

    public TurretSubsystem() {
        configureMotor();
        configureDistanceMap();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Current limits
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // ---------------- PID + Feedforward (YOU MUST TUNE) ----------------
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12;

        flywheelMotor.getConfigurator().apply(config);
    }

    private void configureDistanceMap() {
        // Distance in METERS → RPM
        distanceToRPM.put(1.0, 2500.0);
        distanceToRPM.put(2.0, 3200.0);
        distanceToRPM.put(3.0, 3800.0);
        distanceToRPM.put(4.0, 4500.0);
        distanceToRPM.put(5.0, 5200.0);
    }

    // ---------------- PUBLIC METHODS ----------------

    public void setTargetRPM(double rpm) {
        double rotationsPerSecond = rpm / 60.0;
        flywheelMotor.setControl(
            velocityRequest.withVelocity(rotationsPerSecond)
        );
    }

    public void stop() {
        flywheelMotor.stopMotor();
    }

    public double getRPMForDistance(double distanceMeters) {
        return distanceToRPM.get(distanceMeters);
    }

    public double getCurrentRPM() {
        return flywheelMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", getCurrentRPM());
        SmartDashboard.putNumber("Shooter Voltage", flywheelMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter Current", flywheelMotor.getStatorCurrent().getValueAsDouble());
    }
}