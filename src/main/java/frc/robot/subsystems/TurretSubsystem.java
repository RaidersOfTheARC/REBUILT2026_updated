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

    // ---------------- Outake Phoenixx Constants ----------------
    public static final double MAX_RPM            = 6000.0;
    private static final double MIN_SHOOT_RPM     = 2200.0;
    private static final double MIN_DISTANCE_METERS = 1.0;
    private static final double MAX_DISTANCE_METERS = 6.0;
    private static final int    OutaleP_CAN_ID   = 21;

    // ---------------- Outake Neo Constants ----------------
    private static final int    OutakeNeo_CAN_ID          =  12;
    private static final double OutakeNeo_SPEED_FORWARD   =  0.9;
 
    // ---------------- Hardware ----------------
    private final TalonFX        OutalePMotor    = new TalonFX(OutaleP_CAN_ID);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final DutyCycleOut   dutyCycleRequest = new DutyCycleOut(0);
    private final SparkMax       OutakeNeoMotor = new SparkMax(OutakeNeo_CAN_ID, MotorType.kBrushless);

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

        OutalePMotor.getConfigurator().apply(config);
    }

    // ---------------- OutaleP Methods ----------------

    public void setTargetRPM(double rpm) {
        double clampedRPM = MathUtil.clamp(rpm, 0.0, MAX_RPM);
        OutalePMotor.setControl(velocityRequest.withVelocity(clampedRPM / 60.0));
    }

    public void setManualOutput(double percentOutput) {
        double clampedOutput = MathUtil.clamp(percentOutput, -1.0, 1.0);
        OutalePMotor.setControl(dutyCycleRequest.withOutput(clampedOutput));
    }

    public void stop() {
        OutalePMotor.stopMotor();
    }

    public double getCurrentRPM() {
        return OutalePMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    public double getRPMForDistance(double distanceMeters) {
        double clampedDistance = MathUtil.clamp(distanceMeters, MIN_DISTANCE_METERS, MAX_DISTANCE_METERS);
        double normalized = (clampedDistance - MIN_DISTANCE_METERS)
            / (MAX_DISTANCE_METERS - MIN_DISTANCE_METERS);
        return MIN_SHOOT_RPM + normalized * (MAX_RPM - MIN_SHOOT_RPM);
    }

    // ---------------- OutakeNeo Methods ----------------

    public void Shoot() {
        isRunning = true;
        OutakeNeoMotor.set(OutakeNeo_SPEED_FORWARD);
    }

    public void stopSpinning() {
        isRunning = false;
        OutakeNeoMotor.set(0);
    }

    public boolean isRunning() {
        return isRunning;
    }

    public double getOutakeNeoCurrent() {
        return OutakeNeoMotor.getOutputCurrent();
    }

    // ---------------- Single Merged Periodic ----------------
    @Override
    public void periodic() {
        // OutaleP telemetry
        SmartDashboard.putNumber("Shooter/RPM",     getCurrentRPM());
        SmartDashboard.putNumber("Shooter/Voltage", OutalePMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Current", OutalePMotor.getStatorCurrent().getValueAsDouble());

        // OutakeNeo telemetry
        SmartDashboard.putBoolean("OutakeNeo/IsRunning", isRunning);
        SmartDashboard.putNumber("OutakeNeo/Current",    OutakeNeoMotor.getOutputCurrent());
        SmartDashboard.putNumber("OutakeNeo/Voltage",    OutakeNeoMotor.getBusVoltage());
        SmartDashboard.putNumber("OutakeNeo/Speed",      OutakeNeoMotor.getEncoder().getVelocity());
    }
}
