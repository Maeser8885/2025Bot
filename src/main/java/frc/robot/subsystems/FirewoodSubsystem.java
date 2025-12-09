package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FirewoodSubsystem extends SubsystemBase{
    
    SparkMax firewoodMotor;

    public FirewoodSubsystem() {
        firewoodMotor = new SparkMax(Constants.PreseasonConstants.kFWIntake, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }

    public Command intakeFirewood(){
        return new InstantCommand(() -> {firewoodMotor.set(Constants.PreseasonConstants.fwinSpeed);}, this);
    }

    public Command outtakeFirewood(){
        return new InstantCommand(() -> {firewoodMotor.set(Constants.PreseasonConstants.fwoutSpeed);}, this);
    }

    public Command stopFirewood(){
        return new InstantCommand(() -> {firewoodMotor.set(0);});
    }

    @Override
    public Command getDefaultCommand() {
        // TODO Auto-generated method stub
        return super.getDefaultCommand();
    }
}
