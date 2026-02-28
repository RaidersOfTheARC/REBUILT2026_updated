package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveInputStream;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import java.io.File;
import java.util.List;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  final CommandXboxController driverXbox = new CommandXboxController(0);
  final Joystick flight = new Joystick(2);

  // ---------------- Subsystems ----------------
  private final TurretSubsystem turret = new TurretSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  // ---------------- PhotonVision ----------------
  private final PhotonCamera camera = new PhotonCamera("ZED_2");

  // ---------------- Swerve Drive ----------------
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
    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Nothing", new PathPlannerAuto("Nothing"));
    SmartDashboard.putData("Auto Chooser", autoChooser);

    drivebase.setDefaultCommand(
        drivebase.driveFieldOriented(driveAngularVelocity));
  }

  private void configureBindings() {

    // ---------------- DISTANCE-BASED SHOOTING ----------------
    driverXbox.a()
        .whileTrue(
            new RunCommand(
                () -> {
                  double distance = getAprilTagDistance();

                  if (distance > 0) {
                    double targetRPM = turret.getRPMForDistance(distance);
                    turret.setTargetRPM(targetRPM);
                  } else {
                    turret.stop();
                  }
                },
                turret))
        .onFalse(new InstantCommand(() -> turret.stop(), turret));
  }

  // ---------------- Get Distance From AprilTag ----------------
  private double getAprilTagDistance() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    if (!results.isEmpty()) {
      PhotonPipelineResult result = results.get(results.size() - 1);

      if (result.hasTargets()) {
        return result
            .getBestTarget()
            .getBestCameraToTarget()
            .getTranslation()
            .getNorm();
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