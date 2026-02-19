package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    // ============================================================
    // TUNING CONSTANTS - Edit these values directly when deploying
    // These are the ONLY values that matter; SmartDashboard cannot
    // override them because we force-write every loop.
    // ============================================================
    private static final double INIT_KP          = 0.0001;  // Start low, increase if sluggish
    private static final double INIT_KI          = 0.005;    // Leave at 0 until P+D are tuned
    private static final double INIT_KD          = 0.7;    // Increase to damp oscillation
    private static final double INIT_MAX_OUTPUT  = 0.02;   // Hard ceiling on motor power (0.0 - 1.0)
    private static final double INIT_TOLERANCE   = 5.0;    // Degrees of acceptable error
    private static final double INIT_IZONE       = 5.0;    // Degrees within which I term activates
    private static final double ABSOLUTE_MAX_OUTPUT = 0.15; // Safety cap; dashboard can NEVER exceed this
    // ============================================================

    private final PhotonCamera camera = new PhotonCamera("ZED_2");
    private final SparkMax targetMotor = new SparkMax(25, MotorType.kBrushless);
    private final PIDController pidController = new PIDController(INIT_KP, INIT_KI, INIT_KD);
    private final LinearFilter yawFilter = LinearFilter.singlePoleIIR(0.06, 0.02);

    private boolean hadTargetLastLoop = false;

    public VisionSubsystem() {
        pidController.setSetpoint(0);
        pidController.setTolerance(INIT_TOLERANCE);
        pidController.setIZone(INIT_IZONE);

        // Force-write initial values to dashboard on every deploy,
        // overwriting any stale values from previous sessions.
        pushDefaultsToDashboard();
    }

    @Override
    public void periodic() {
        // Read from dashboard so values can be tweaked live during testing.
        // The ABSOLUTE_MAX_OUTPUT cap below ensures safety regardless of dashboard input.
        double kP        = SmartDashboard.getNumber("Vision/kP",         INIT_KP);
        double kI        = SmartDashboard.getNumber("Vision/kI",         INIT_KI);
        double kD        = SmartDashboard.getNumber("Vision/kD",         INIT_KD);
        double maxOutput = SmartDashboard.getNumber("Vision/MaxOutput",  INIT_MAX_OUTPUT);
        double tolerance = SmartDashboard.getNumber("Vision/ToleranceDeg", INIT_TOLERANCE);

        // Safety cap - clamp maxOutput regardless of what the dashboard says
        maxOutput = Math.min(Math.abs(maxOutput), ABSOLUTE_MAX_OUTPUT);

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setTolerance(tolerance);

        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            if (!hadTargetLastLoop) {
                // Target just reacquired - reset to prevent integrator windup kick
                pidController.reset();
                yawFilter.reset();
            }
            hadTargetLastLoop = true;

            double rawYaw      = result.getBestTarget().getYaw();
            double filteredYaw = yawFilter.calculate(rawYaw);

            SmartDashboard.putNumber("Vision/Raw Yaw",      rawYaw);
            SmartDashboard.putNumber("Vision/Filtered Yaw", filteredYaw);
            SmartDashboard.putNumber("Vision/Max Output (effective)", maxOutput);

            if (pidController.atSetpoint()) {
                targetMotor.set(0);
                SmartDashboard.putString("Vision/State",        "ON TARGET");
                SmartDashboard.putNumber("Vision/Motor Output", 0);
            } else {
                double output = pidController.calculate(filteredYaw);
                output = MathUtil.clamp(output, -maxOutput, maxOutput);

                // Negative because motor must spin opposite to yaw direction
                targetMotor.set(-output);
                SmartDashboard.putString("Vision/State",        "TRACKING");
                SmartDashboard.putNumber("Vision/Motor Output", -output);
            }

        } else {
            targetMotor.set(0);
            hadTargetLastLoop = false;
            yawFilter.reset();
            SmartDashboard.putString("Vision/State",        "NO TARGET");
            SmartDashboard.putNumber("Vision/Motor Output", 0);
            SmartDashboard.putNumber("Vision/Raw Yaw",      0);
            SmartDashboard.putNumber("Vision/Filtered Yaw", 0);
        }
    }

    /**
     * Force-writes all tuning defaults to SmartDashboard.
     * Call this in the constructor to overwrite stale saved values from previous deploys.
     */
    private void pushDefaultsToDashboard() {
        SmartDashboard.putNumber("Vision/kP",           INIT_KP);
        SmartDashboard.putNumber("Vision/kI",           INIT_KI);
        SmartDashboard.putNumber("Vision/kD",           INIT_KD);
        SmartDashboard.putNumber("Vision/MaxOutput",    INIT_MAX_OUTPUT);
        SmartDashboard.putNumber("Vision/ToleranceDeg", INIT_TOLERANCE);
    }
}















/*
package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera("ZED_2");
    private final SparkMax targetMotor = new SparkMax(25, MotorType.kBrushless);
    
    // PID Controller - Start with 0 for I and D, and a very low P
    private final PIDController pidController = new PIDController(0.0008, 0.01, .00001);
    
    // LOW PASS FILTER: This smooths out jumpy data from the camera
    // 0.1 is the time constant; increase for more smoothing, decrease for faster response.
    private final LinearFilter yawFilter = LinearFilter.singlePoleIIR(0.3, 0.02);

    public VisionSubsystem() {
        SmartDashboard.putNumber("Vision/kP", 0.008);
        SmartDashboard.putNumber("Vision/kD", 0.0005);
        SmartDashboard.putNumber("Vision/kFF", 0.02); // Lowered so it doesn't overpower the clamp
        SmartDashboard.putNumber("Vision/MaxOutput", 0.04);

        pidController.setSetpoint(0);
        pidController.setTolerance(0.5); // 0.1 is too small for camera noise; 0.5 is steadier
    }

    @Override
    public void periodic() {
        pidController.setP(SmartDashboard.getNumber("Vision/kP", 0.008));
        pidController.setD(SmartDashboard.getNumber("Vision/kD", 0.0005));
        double kFF = SmartDashboard.getNumber("Vision/kFF", 0.02);
        double maxOutput = SmartDashboard.getNumber("Vision/MaxOutput", 0.15);

        PhotonPipelineResult result = camera.getLatestResult();
        
        if (result.hasTargets()) {
            // Get raw yaw and pass it through the filter to remove jitter
            double rawYaw = result.getBestTarget().getYaw();
            double filteredYaw = yawFilter.calculate(rawYaw);
            
            double pidOutput = pidController.calculate(filteredYaw);
            
            // Feedforward should be SMALLER than your PID output
            double feedforward = Math.signum(pidOutput) * kFF;
            
            double totalSpeed = pidOutput + feedforward;

            // Ensure we don't exceed the physical limits
            totalSpeed = MathUtil.clamp(totalSpeed, -maxOutput, maxOutput);
            
            if (pidController.atSetpoint()) {
                targetMotor.set(0);
            } else {
                // Using negative because you want to spin OPPOSITE of yaw
                targetMotor.set(-totalSpeed);
            }

            SmartDashboard.putNumber("Vision/Filtered Yaw", filteredYaw);
            SmartDashboard.putNumber("Vision/Motor Speed", -totalSpeed);
        } else {
            targetMotor.set(0);
            yawFilter.reset(); // Reset filter when target is lost
        }
    }
}
*/