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
import java.util.List;

public class VisionSubsystem extends SubsystemBase {

    // ============================================================
    // TUNING CONSTANTS - Edit only these values between deploys
    // ============================================================
    private static final double INIT_KP             = 0.0002; // .0003
    private static final double INIT_KI             = 0.2; //.5
    private static final double INIT_KD             = 0.5; // .7
    private static final double INIT_MAX_OUTPUT     = 0.04; //.08
    private static final double INIT_TOLERANCE      = 3.0;
    private static final double INIT_IZONE          = 5.0;
    private static final double ABSOLUTE_MAX_OUTPUT = 0.18; // dashboard can NEVER exceed this
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
        pushDefaultsToDashboard();
    }

    @Override
    public void periodic() {
        double kP        = SmartDashboard.getNumber("Vision/kP",           INIT_KP);
        double kI        = SmartDashboard.getNumber("Vision/kI",           INIT_KI);
        double kD        = SmartDashboard.getNumber("Vision/kD",           INIT_KD);
        double maxOutput = SmartDashboard.getNumber("Vision/MaxOutput",    INIT_MAX_OUTPUT);
        double tolerance = SmartDashboard.getNumber("Vision/ToleranceDeg", INIT_TOLERANCE);

        // Hard safety cap regardless of dashboard value
        maxOutput = Math.min(Math.abs(maxOutput), ABSOLUTE_MAX_OUTPUT);

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setTolerance(tolerance);

        // ---------------------------------------------------------------
        // THE CRITICAL FIX: getAllUnreadResults() not getLatestResult()
        //
        // getLatestResult() returns the same cached frame every loop until
        // a brand new frame arrives from the camera. Once the tag appears
        // centered in that cached frame, the PID sees zero error forever
        // and stops correcting — even when the tag physically moves.
        //
        // getAllUnreadResults() only returns frames produced SINCE your
        // last call, so every correction is based on genuinely fresh data.
        // ---------------------------------------------------------------
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (!results.isEmpty()) {
            // Use only the most recent frame from this batch
            PhotonPipelineResult result = results.get(results.size() - 1);

            if (result.hasTargets()) {
                if (!hadTargetLastLoop) {
                    // Just reacquired — reset to avoid integrator windup kick
                    pidController.reset();
                    yawFilter.reset();
                }
                hadTargetLastLoop = true;

                double rawYaw      = result.getBestTarget().getYaw();
                double filteredYaw = yawFilter.calculate(rawYaw);

                SmartDashboard.putNumber("Vision/Raw Yaw",                rawYaw);
                SmartDashboard.putNumber("Vision/Filtered Yaw",           filteredYaw);
                SmartDashboard.putNumber("Vision/Max Output (effective)", maxOutput);

                if (pidController.atSetpoint()) {
                    targetMotor.set(0);
                    SmartDashboard.putString("Vision/State",        "ON TARGET");
                    SmartDashboard.putNumber("Vision/Motor Output", 0);
                } else {
                    double output = pidController.calculate(filteredYaw);
                    output = MathUtil.clamp(output, -maxOutput, maxOutput);

                    targetMotor.set(-output);
                    SmartDashboard.putString("Vision/State",        "TRACKING");
                    SmartDashboard.putNumber("Vision/Motor Output", -output);
                }

            } else {
                // Fresh frame arrived but no tag visible
                stopMotor();
            }

        } else {
            // No new camera frames this loop cycle.
            // Do NOT re-run PID on stale data — just hold state.
            if (!hadTargetLastLoop) {
                targetMotor.set(0);
            }
            SmartDashboard.putString("Vision/State", "WAITING FOR FRAME");
        }
    }

    private void stopMotor() {
        targetMotor.set(0);
        hadTargetLastLoop = false;
        yawFilter.reset();
        SmartDashboard.putString("Vision/State",        "NO TARGET");
        SmartDashboard.putNumber("Vision/Motor Output", 0);
        SmartDashboard.putNumber("Vision/Raw Yaw",      0);
        SmartDashboard.putNumber("Vision/Filtered Yaw", 0);
    }

    private void pushDefaultsToDashboard() {
        SmartDashboard.putNumber("Vision/kP",           INIT_KP);
        SmartDashboard.putNumber("Vision/kI",           INIT_KI);
        SmartDashboard.putNumber("Vision/kD",           INIT_KD);
        SmartDashboard.putNumber("Vision/MaxOutput",    INIT_MAX_OUTPUT);
        SmartDashboard.putNumber("Vision/ToleranceDeg", INIT_TOLERANCE);
    }
}



/*package frc.robot.subsystems;

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
    private static final double INIT_KI          = 0.1;    // Leave at 0 until P+D are tuned
    private static final double INIT_KD          = 0.7;    // Increase to damp oscillation
    private static final double INIT_MAX_OUTPUT  = 0.04;   // Hard ceiling on motor power (0.0 - 1.0)
    private static final double INIT_TOLERANCE   = 3.0;    // Degrees of acceptable error
    private static final double INIT_IZONE       = 5.0;    // Degrees within which I term activates
    private static final double ABSOLUTE_MAX_OUTPUT = 0.18; // Safety cap; dashboard can NEVER exceed this
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


    private void pushDefaultsToDashboard() {
        SmartDashboard.putNumber("Vision/kP",           INIT_KP);
        SmartDashboard.putNumber("Vision/kI",           INIT_KI);
        SmartDashboard.putNumber("Vision/kD",           INIT_KD);
        SmartDashboard.putNumber("Vision/MaxOutput",    INIT_MAX_OUTPUT);
        SmartDashboard.putNumber("Vision/ToleranceDeg", INIT_TOLERANCE);
    }
}*/