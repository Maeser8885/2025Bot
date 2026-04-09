// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import swervelib.SwerveModule;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

    private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

    // --- Controllers ---
    // We use two Logitech F310 gamepads (in X-input mode, which is Xbox-compatible).
    // CommandXboxController is the command-based wrapper — it gives you Trigger objects for each
    // button so you can attach commands with .onTrue(), .whileTrue(), etc.
    // Port numbers must match how the controllers are plugged into the Driver Station laptop.
    // IMPORTANT: The switch on the bottom of each F310 must be set to "X" (not "D").
    private final CommandXboxController driverController = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);   // Port 0

    /**
     * This function is run when the robot is first startesd up and should be used
     * for any
     * initialization code.
     */
    public Robot() {
        // --- Print absolute encoder positions at startup for calibration ---
        for (SwerveModule module : drivetrain.getSwerveDrive().getModules()) {
            String name = module.getConfiguration().name;
            double raw = module.getRawAbsolutePosition();
            System.out.println("Encoder [" + name + "] raw absolute position: " + raw);
            SmartDashboard.putNumber("Calibration/" + name + " Raw Abs", raw);
        }

        double speed = DrivetrainConstants.kMaxSpeedMetersPerSecond;
        double maxRotSpeed = DrivetrainConstants.kMaxAngularSpeedRadiansPerSecond;
        double calibRotSpeed = maxRotSpeed * 0.25;

        // Default command: joystick driving (also stops the bot when sticks are centered)
        double deadband = Constants.OperatorConstants.kJoystickDeadband;
        drivetrain.setDefaultCommand(drivetrain.run(() -> {
            double forward = -MathUtil.applyDeadband(driverController.getLeftY(), deadband);
            double strafe = -MathUtil.applyDeadband(driverController.getLeftX(), deadband);
            double rot = -MathUtil.applyDeadband(driverController.getRightX(), deadband);

            drivetrain.drive(
                    new Translation2d(forward * speed, strafe * speed),
                    rot * maxRotSpeed,
                    drivetrain.isFieldOriented());
        }));

        // Start = toggle field-oriented / robot-oriented
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.toggleFieldOriented()));

        // Back = zero gyro
        driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.zeroGyro()));

        // --- Calibration button bindings ---

        // Y = forward, A = backward (robot-oriented)
        driverController.y().whileTrue(drivetrain.run(() -> drivetrain.drive(new Translation2d(speed, 0), 0, false)));
        driverController.a().whileTrue(drivetrain.run(() -> drivetrain.drive(new Translation2d(-speed, 0), 0, false)));

        // X = strafe left, B = strafe right (robot-oriented)
        driverController.x().whileTrue(drivetrain.run(() -> drivetrain.drive(new Translation2d(0, speed), 0, false)));
        driverController.b().whileTrue(drivetrain.run(() -> drivetrain.drive(new Translation2d(0, -speed), 0, false)));

        // LB = rotate counter-clockwise, RB = rotate clockwise
        driverController.leftBumper().whileTrue(drivetrain.run(() -> drivetrain.drive(new Translation2d(0, 0), calibRotSpeed, false)));
        driverController.rightBumper().whileTrue(drivetrain.run(() -> drivetrain.drive(new Translation2d(0, 0), -calibRotSpeed, false)));
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        drivetrain.zeroGyro();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {

    }
}
