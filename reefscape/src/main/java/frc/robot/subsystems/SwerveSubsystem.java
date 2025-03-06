// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlignmentConstants;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;

public class SwerveSubsystem extends SubsystemBase {
  
  private final SwerveDrive  swerveDrive;
  
  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem(){
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.MAX_SPEED);
      //swerveDrive.getGyro().setOffset(swerveDrive.getGyroRotation3d().unaryMinus());
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> swerveDrive.setChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                  new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                  new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
          ),
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  /**
   * Angular velocity drive command.
   *
   * @return a command
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY)
  {
    return this.run(() -> {
      // Make the robot move
      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                translationY.getAsDouble()), 0.7);
        
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(), headingX.getAsDouble(), headingY.getAsDouble(), swerveDrive.getOdometryHeading().getRadians(), swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOrientedCommand(Supplier<ChassisSpeeds> velocity) {
    return this.run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  /*
   * Command to align the robot to be parallel to the targeted AprilTag
   * CAUTION: May need to flip the sign of dtheta for correct behavior
   */
  public Command alignToTagCommand(VisionSubsystem vision)
  {
    return this.run(() -> {
      boolean isDefault = true; // is the Limelight returning default value?
      double[] pose = vision.getCameraPose();

      for (double value : pose) {
          if (value != 0.0) {
              isDefault = false; // Found a non-zero value, so not default value
              break; // No need to check further
          }
      }
    
      if (!isDefault)
      {
        double dtheta = pose[4]; // yaw offset (MIGHT NEED A MINUS SIGN AND/OR a +/- 180 degrees)
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0, 0, -AlignmentConstants.kPSwerveAlignTheta * dtheta);
        driveFieldOriented(desiredSpeeds);
      }
    });

  }

  /*
   * Command to align the robot to be parallel to the targeted AprilTag
   * CAUTION: May need to flip the sign of dtheta for correct behavior
   */
  public Command moveToTagCommand(double yOffset, VisionSubsystem vision) {
    return this.run(() -> {

      boolean isDefault = true; // is the Limelight returning default value?
      double[] pose = vision.getCameraPose();

      for (double value : pose) {
          if (value != 0.0) {
              isDefault = false; // Found a non-zero value, so not default value
              break; // No need to check further
          }
      }

      if (!isDefault) { // only proceed if limelight not reading default value
        double dz = pose[2]; // approximate distance to apriltag (in meters)
        double dx = yOffset - pose[0]; // approximate longitudal distance to target location (in meters) 

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(-AlignmentConstants.kPSwerveAlignZ * dz, AlignmentConstants.kPSwerveAlignX * dx, 0);
        driveFieldOriented(desiredSpeeds);
      }

    });
  }

  //set the gyro to zero
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }

   /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

   /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
