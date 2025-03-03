// // Copyright (c) 2025 FRC 6328
// // http://github.com/Mechanical-Advantage
// //
// // Use of this source code is governed by an MIT-style
// // license that can be found in the LICENSE file at
// // the root directory of this project.

// package frc.robot.commands;

// import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
// import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.SwerveConstants;
// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.Swerve;
// import frc.robot.util.GeomUtil;
// import java.util.function.DoubleSupplier;
// import java.util.function.Supplier;

// public class ReefAlignmentPID extends Command {
//   private double drivekP = 0.75;
//   private double drivekD = 0.0;
//   private double thetakP = 4.0;
//   private double thetakD = 0.0;
//   private double driveMaxVelocity = 3.8;
//   private double driveMaxAcceleration = 3.0;
//   private double thetaMaxVelocity = Units.degreesToRadians(1);
//   private double thetaMaxAcceleration = 8;
//   private double driveTolerance = 0.01;
//   private double thetaTolerance = Units.degreesToRadians(1.0);
//   private double ffMinRadius = 0.05;
//   private double ffMaxRadius = 0.1;

//   private final Swerve drive = TunerConstants.createDrivetrain();
//   private final Supplier<Pose2d> target;

//   private SwerveRequest.FieldCentric fieldOriented =
//       new SwerveRequest.FieldCentric()
//           .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
//           .withSteerRequestType(SteerRequestType.Position);

//   private final ProfiledPIDController driveController =
//       new ProfiledPIDController(
//           drivekP,
//           0.0,
//           drivekD,
//           new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration),
//           0.02);
//   private final ProfiledPIDController thetaController =
//       new ProfiledPIDController(
//           thetakP,
//           0.0,
//           thetakD,
//           new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration),
//           0.02);

//   private Translation2d lastSetpointTranslation = new Translation2d();
//   private double driveErrorAbs = 0.0;
//   private double thetaErrorAbs = 0.0;
//   private boolean running = false;
//   private Pose2d robotPose = drive.getState().Pose;
//   private Supplier<Pose2d> robot = () -> robotPose;

//   private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
//   private DoubleSupplier omegaFF = () -> 0.0;

//   public ReefAlignmentPID(Swerve drive, Supplier<Pose2d> target) {
//     this.target = target;

//     // Enable continuous input for theta controller
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);

//     addRequirements(drive);
//   }

//   public ReefAlignmentPID(Supplier<Pose2d> target) {
//     this.target = target;
//   }

//   // public ReefAlignmentPID(
//   //     Swerve drive,
//   //     Supplier<Pose2d> target,
//   //     Supplier<Pose2d> robot,
//   //     Supplier<Translation2d> linearFF,
//   //     DoubleSupplier omegaFF) {
//   //   this(target, robot);
//   //   this.linearFF = linearFF;
//   //   this.omegaFF = omegaFF;
//   // }

//   @Override
//   public void initialize() {
//     Pose2d currentPose = robot.get();
//     ChassisSpeeds fieldVelocity = drive.getState().Speeds;
//     Translation2d linearFieldVelocity =
//         new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
//     driveController.reset(
//         currentPose.getTranslation().getDistance(target.get().getTranslation()),
//         Math.min(
//             0.0,
//             -linearFieldVelocity
//                 .rotateBy(
//                     target
//                         .get()
//                         .getTranslation()
//                         .minus(currentPose.getTranslation())
//                         .getAngle()
//                         .unaryMinus())
//                 .getX()));
//     thetaController.reset(
//         currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
//     lastSetpointTranslation = currentPose.getTranslation();
//   }

//   @Override
//   public void execute() {
//     running = true;

//     // // Update from tunable numbers
//     // if (driveMaxVelocity.hasChanged(hashCode())
//     //     || driveMaxVelocitySlow.hasChanged(hashCode())
//     //     || driveMaxAcceleration.hasChanged(hashCode())
//     //     || driveTolerance.hasChanged(hashCode())
//     //     || thetaMaxVelocity.hasChanged(hashCode())
//     //     || thetaMaxAcceleration.hasChanged(hashCode())
//     //     || thetaTolerance.hasChanged(hashCode())
//     //     || drivekP.hasChanged(hashCode())
//     //     || drivekD.hasChanged(hashCode())
//     //     || thetakP.hasChanged(hashCode())
//     //     || thetakD.hasChanged(hashCode())) {
//     //   driveController.setP(drivekP.get());
//     //   driveController.setD(drivekD.get());
//     //   driveController.setConstraints(
//     //       new TrapezoidProfile.Constraints(driveMaxVelocity.get(),
// driveMaxAcceleration.get()));
//     //   driveController.setTolerance(driveTolerance.get());
//     //   thetaController.setP(thetakP.get());
//     //   thetaController.setD(thetakD.get());
//     //   thetaController.setConstraints(
//     //       new TrapezoidProfile.Constraints(thetaMaxVelocity.get(),
// thetaMaxAcceleration.get()));
//     //   thetaController.setTolerance(thetaTolerance.get());
//     // }

//     // Get current pose and target pose
//     Pose2d currentPose = robot.get();
//     Pose2d targetPose = target.get();

//     // Calculate drive speed
//     double currentDistance =
// currentPose.getTranslation().getDistance(targetPose.getTranslation());
//     double ffScaler =
//         MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
//     driveErrorAbs = currentDistance;
//     driveController.reset(
//         lastSetpointTranslation.getDistance(targetPose.getTranslation()),
//         driveController.getSetpoint().velocity);
//     double driveVelocityScalar =
//         driveController.getSetpoint().velocity * ffScaler
//             + driveController.calculate(driveErrorAbs, 0.0);
//     if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
//     lastSetpointTranslation =
//         new Pose2d(
//                 targetPose.getTranslation(),
//                 currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
//             .transformBy(GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
//             .getTranslation();

//     // Calculate theta speed
//     double thetaVelocity =
//         thetaController.getSetpoint().velocity * ffScaler
//             + thetaController.calculate(
//                 currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
//     thetaErrorAbs =
//         Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
//     if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

//     Translation2d driveVelocity =
//         new Pose2d(
//                 new Translation2d(),
//                 currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
//             .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
//             .getTranslation();

//     // Scale feedback velocities by input ff
//     final double linearS = linearFF.get().getNorm() * 3.0;
//     final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
//     driveVelocity =
//         driveVelocity.interpolate(
//             linearFF.get().times(SwerveConstants.maxTranslationalSpeed.magnitude()), linearS);
//     // is the magnitude() right???
//     thetaVelocity =
//         MathUtil.interpolate(
//             thetaVelocity,
//             omegaFF.getAsDouble() * SwerveConstants.maxRotationalSpeed.baseUnitMagnitude(),
//             thetaS);

//     drive.setControl(
//         fieldOriented
//             .withVelocityX(driveVelocity.getX())
//             .withVelocityY(driveVelocity.getY())
//             .withRotationalRate(thetaVelocity));
//   }

//   @Override
//   public void end(boolean interrupted) {
//     drive.setControl(fieldOriented.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
//     running = false;
//   }

//   /** Checks if the robot is stopped at the final pose. */
//   public boolean atGoal() {
//     return running && driveController.atGoal() && thetaController.atGoal();
//   }

//   /** Checks if the robot pose is within the allowed drive and theta tolerances. */
//   public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
//     return running
//         && Math.abs(driveErrorAbs) < driveTolerance
//         && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
//   }
// }
