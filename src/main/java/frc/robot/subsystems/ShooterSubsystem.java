package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{

    SparkMax trMotor;
    SparkMax tlMotor;
    SparkMax brMotor;
    SparkMax blMotor;

    float currentSpeed = 0.2f;

    public ShooterSubsystem() {

        trMotor = new SparkMax(Constants.PreseasonConstants.trShooterPort, MotorType.kBrushless);

        SparkMaxConfig followConfig = new SparkMaxConfig();        
        
        SparkMaxConfig invertedConfig = new SparkMaxConfig();
        invertedConfig.inverted(true);

        brMotor = new SparkMax(Constants.PreseasonConstants.brShooterPort, MotorType.kBrushless);
        brMotor.configure(invertedConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followConfig.follow(brMotor);
        invertedConfig.follow(brMotor, true);

        tlMotor = new SparkMax(Constants.PreseasonConstants.tlShooterPort, MotorType.kBrushless);
        tlMotor.configure(invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        blMotor = new SparkMax(Constants.PreseasonConstants.blShooterPort, MotorType.kBrushless);
        blMotor.configure(followConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);       
    }

    @Override
    public void periodic() {
        
        super.periodic();
    }

    public Command outtake(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> { brMotor.set(Constants.PreseasonConstants.shooterOutSpeed);}, this),
            new WaitCommand(0.3),
            new InstantCommand(() -> { trMotor.set(Constants.PreseasonConstants.shooterOutSpeed * -1); }, this)
        );
    }
    

    public Command stop(){
        return new InstantCommand(() -> {
            trMotor.set(0);
            brMotor.set(0);
        }, this);
    }

    public Command changeSpeed(float speed){
        return new InstantCommand(() -> {currentSpeed = speed;}, this);
    }

    @Override
    public Command getDefaultCommand() {
        // TODO Auto-generated method stub
        return super.getDefaultCommand();
    }
}
