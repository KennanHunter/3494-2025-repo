package frc.robot.subsystems.SuperScructure;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator  extends SubsystemBase{
    private SparkMax leaderMotor;
    private SparkMaxConfig leaderConfig;

    private SparkMax followerMotor;
    private SparkMaxConfig followerConfig;

    
    public Elevator (){
        leaderConfig.idleMode(IdleMode.kCoast);
        followerConfig.follow(leaderMotor);

        leaderMotor.configure(
            leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotor.configure(
            followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
