// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

import java.io.File;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.VisionSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer{

  //Allows for autons to be decided at the match instead of requiring pushing prior
  private final SendableChooser<Command> autoChooser;

  //Xbox controller and flight stick intialization, make sure joystick order matches these ids in FRC Driver Station
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController driverXbox2 = new CommandXboxController(1);
  final Joystick flight = new Joystick(2);
  private final VisionSubsystem m_vision = new VisionSubsystem();

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
  
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  //THIS IS THE PREVIOUS VERSION OF THE driveAngularVelocity. If new drive code has issues, revert to this
  /*SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                 () -> driverXbox.getLeftY() * 0.75,
                                                                 () -> driverXbox.getLeftX() * 0.75)
                                                             .withControllerRotationAxis(driverXbox::getRightX)
                                                             .deadband(OperatorConstants.DEADBAND)
                                                             .scaleTranslation(0.8)
                                                             .allianceRelativeControl(true);
  */
  //THIS IS THE NEW VERSION OF THE driveAngularVelocity.
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                 () -> driverXbox.getLeftY() * 0.75,
                                                                 () -> driverXbox.getLeftX() * 0.75)
    .withControllerRotationAxis(() -> {
        double stickX = driverXbox.getRightX()*-1;

        // Allows for normal rotation with right stick if stick is not neutral
        if (Math.abs(driverXbox.getRightX()) > 0.05) {
            return stickX;
        }

        // Uses triggers for slower rotation when right stick is neutral (helps with alignment)
        double clockwise = driverXbox.getRightTriggerAxis();
        double counterclockwise = driverXbox.getLeftTriggerAxis();

        double slowRotateSpeed = 0.3;  // tune this value for slower/faster spin as needed at comp
        return (clockwise - counterclockwise) * slowRotateSpeed;
    })
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);


  /*SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                            () -> flight.getY() * 1,
                                                            () -> flight.getX() * 1)
                                                        .withControllerRotationAxis(flight::getTwist)
                                                        .deadband(OperatorConstants.DEADBAND)
                                                        .allianceRelativeControl(true);
  */
  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
  //  */
  // SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
  //                                                                                            driverXbox::getRightY)
  //                                                          .headingWhile(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(flight::getX,
                          flight::getY)
                         .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);
                                                                    
  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                    () -> -flight.getY(),
                                                                    () -> -flight.getX())
                                                                .withControllerRotationAxis(() -> flight.getZ())
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(0.8)
                                                                .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  flight.getX(
                                                                                                                      ) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  flight.getY(
                                                                                                                      ) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(){
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Taxi", new PathPlannerAuto("Taxi"));
    autoChooser.addOption("Nothing", new PathPlannerAuto("Nothing"));
    //Add every auton to the autochooser

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings(){
    //Different settings for swerve drive
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);


    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()){}

    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Take from auto chooser
    return autoChooser.getSelected();
    // return null;
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
