package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transfer extends SubsystemBase {

    private static final int    TRANSFER_CAN_ID        = 12;
    private static final double TRANSFER_SPEED_FORWARD =  1.0;

    private final SparkMax transferMotor = new SparkMax(TRANSFER_CAN_ID, MotorType.kBrushless);
    private boolean isTransferring = false;

    public Transfer() {
        configureTransfer();
    }

    private void configureTransfer() {
        SparkMaxConfig config = new SparkMaxConfig();
        transferMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    // Renamed from Transfer() — was being treated as a constructor
    public void runTransfer() {
        isTransferring = true;
        transferMotor.set(TRANSFER_SPEED_FORWARD);
    }

    public void stopTransferring() {
        isTransferring = false;
        transferMotor.set(0);
    }

    public boolean isTransferring() { return isTransferring; }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Transfer/IsRunning", isTransferring);
        SmartDashboard.putNumber("Transfer/Current",    transferMotor.getOutputCurrent());
        SmartDashboard.putNumber("Transfer/Voltage",    transferMotor.getBusVoltage());
    }
}