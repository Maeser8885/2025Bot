// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    public static final int DRIVER_CONTROLLER_PORT = 1;
    public static final double JOYSTICK_DEADBAND = 0.08;


    private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

    // --- Controllers ---
    // We use two Logitech F310 gamepads (in X-input mode, which is Xbox-compatible).
    // CommandXboxController is the command-based wrapper — it gives you Trigger objects for each
    // button so you can attach commands with .onTrue(), .whileTrue(), etc.
    // Port numbers must match how the controllers are plugged into the Driver Station laptop.
    // IMPORTANT: The switch on the bottom of each F310 must be set to "X" (not "D").
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);   // Port 1

    /**
     * This function is run when the robot is first startesd up and should be used
     * for any
     * initialization code.
     */
    public Robot() {
        drivetrain.publishStats();

        // Default command: joystick driving (also stops the bot when sticks are centered)
        drivetrain.setDefaultCommand(drivetrain.run(() -> {
            double forward = -MathUtil.applyDeadband(driverController.getLeftY(), JOYSTICK_DEADBAND);
            double strafe = -MathUtil.applyDeadband(driverController.getLeftX(), JOYSTICK_DEADBAND);
            double rot = -MathUtil.applyDeadband(driverController.getRightX(), JOYSTICK_DEADBAND);
            drivetrain.drive(forward, strafe, rot);
        }));

        // Start = toggle field-oriented / robot-oriented
        driverController.start().onTrue(drivetrain.runOnce(drivetrain::toggleFieldOriented));

        // Back = zero gyro
        driverController.back().onTrue(drivetrain.runOnce(drivetrain::zeroGyro));

        // --- Calibration button bindings ---
        // Y = forward, A = backward
        driverController.y().whileTrue(drivetrain.run(() -> drivetrain.drive(1, 0, 0)));
        driverController.a().whileTrue(drivetrain.run(() -> drivetrain.drive(-1, 0, 0)));

        // X = strafe left, B = strafe right
        driverController.x().whileTrue(drivetrain.run(() -> drivetrain.drive(0, 1, 0)));
        driverController.b().whileTrue(drivetrain.run(() -> drivetrain.drive(0, -1, 0)));

        // LB = rotate counter-clockwise, RB = rotate clockwise
        driverController.leftBumper().whileTrue(drivetrain.run(() -> drivetrain.drive(0, 0, 1)));
        driverController.rightBumper().whileTrue(drivetrain.run(() -> drivetrain.drive(0, 0, -1)));

        drivetrain.zeroGyro();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {

    }
}
