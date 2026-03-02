package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  // Controllers
  // Port 0 — Driver    : swerve only
  // Port 1 — Secondary : flywheel, flywheelspin bumpers, apriltag X, dpad intake lift
  final CommandXboxController driverXbox    = new CommandXboxController(0);
  final CommandXboxController secondaryXbox = new CommandXboxController(1);

  // Subsystems
  private final TurretSubsystem turret    = new TurretSubsystem();
  private final IntakeSubsystem intake    = new IntakeSubsystem();
  private final Transfer        transfer  = new Transfer();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  private final VisionSubsystem vision    = new VisionSubsystem();

  private final PhotonCamera camera = new PhotonCamera("ZED_2");

  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> driverXbox.getLeftY() * 0.75,
              () -> driverXbox.getLeftX() * 0.75)
          .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    // ── PathPlanner Named Commands ────────────────────────────────────────
    // MUST be registered before AutoBuilder.buildAutoChooser()
    // These names must match EXACTLY what you type in PathPlanner

    // Spins flywheel up to shooting speed — trigger this at path START
    // so it's up to speed by the time robot reaches shooting position
    NamedCommands.registerCommand("SpinUpFlywheel",
        new InstantCommand(() -> turret.setTargetRPM(4000), turret));

    // Runs flywheelspin + transfer together to fire the note — trigger at END of path
    NamedCommands.registerCommand("Shoot",
        new RunCommand(
            () -> {
              turret.turnRight();
              transfer.runTransfer();
            },
            turret, transfer)
        .withTimeout(1.5)); // runs for 1.5 seconds then moves on

    // Stops everything after shooting
    NamedCommands.registerCommand("StopAll",
        new InstantCommand(
            () -> {
              turret.stop();
              turret.stopSpinning();
              transfer.stopTransferring();
            },
            turret, transfer));

    // ─────────────────────────────────────────────────────────────────────

    configureBindings();
    configureDefaultCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureDefaultCommands() {

    // Port 0 — swerve only
    drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));

    // Port 1 left stick Y — flywheel manual
    turret.setDefaultCommand(
        new RunCommand(
            () -> {
              double input = MathUtil.applyDeadband(
                  -secondaryXbox.getLeftY(), OperatorConstants.DEADBAND);

              SmartDashboard.putNumber("Shooter/StickInput", input);

              if (Math.abs(input) > 0.0) {
                turret.setManualOutput(input * 0.6);
              } else {
                turret.stop();
              }
            },
            turret));

    // Intake default — stopped unless bindings override
    intake.setDefaultCommand(
        new RunCommand(
            () -> {
              intake.stop();
              intake.stopLifting();
            },
            intake));
  }

  private void configureBindings() {

    // ── Port 1 — Flywheel ────────────────────────────────────────────────

    secondaryXbox.x()
        .whileTrue(
            new RunCommand(
                () -> {
                  double distance = getAprilTagDistance();
                  if (distance > 0) {
                    turret.setTargetRPM(turret.getRPMForDistance(distance));
                    SmartDashboard.putNumber("Shooter/TargetRPM", turret.getRPMForDistance(distance));
                    SmartDashboard.putNumber("Shooter/Distance",  distance);
                  } else {
                    turret.stop();
                  }
                },
                turret))
        .onFalse(new InstantCommand(() -> turret.stop(), turret));

    // ── Port 1 — Flywheelspin bumpers ────────────────────────────────────

    secondaryXbox.rightBumper()
        .whileTrue(new RunCommand(() -> turret.turnRight(), turret))
        .onFalse(new InstantCommand(() -> turret.stopSpinning(), turret));

    secondaryXbox.leftBumper()
        .whileTrue(new RunCommand(() -> turret.turnLeft(), turret))
        .onFalse(new InstantCommand(() -> turret.stopSpinning(), turret));

    // ── Port 1 — Transfer Y button ───────────────────────────────────────
    // Fixed: was transfer.Transfer() and transfer.stopTransfering()
    secondaryXbox.y()
        .whileTrue(new RunCommand(() -> transfer.runTransfer(), transfer))
        .onFalse(new InstantCommand(() -> transfer.stopTransferring(), transfer));

    // ── Port 1 — Intake lift dpad ─────────────────────────────────────────

    secondaryXbox.povUp()
        .whileTrue(new RunCommand(() -> intake.raiseUp(), intake))
        .onFalse(new InstantCommand(() -> intake.stopLifting(), intake));

    secondaryXbox.povDown()
        .whileTrue(new RunCommand(() -> intake.raiseDown(), intake))
        .onFalse(new InstantCommand(() -> intake.stopLifting(), intake));

    secondaryXbox.povLeft()
        .whileTrue(new RunCommand(() -> intake.holdAt90(), intake))
        .onFalse(new InstantCommand(() -> intake.releaseHold(), intake));
  }

  private double getAprilTagDistance() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      PhotonPipelineResult result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        return result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
      }
    }
    return -1;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}