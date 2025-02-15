package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class AutoAlignDesitationDeterminer {
    public static Supplier<Pose2d> destination(Pose2d robotPosition, boolean leftSide){
        Supplier<Pose2d> targetSupplier =
        () -> {
            double distance;
            double minDistance = 99999999;
            Optional<Alliance> ally = DriverStation.getAlliance();
            int minIndex = -1;
            for(int i = 0; i< Constants.Field.Reef.sideLocations.length; i++){
                
                if(ally.get() == DriverStation.Alliance.Red){
                    distance = getDistance(robotPosition, transform2red(Constants.Field.Reef.sideLocations[i]));
                }
                else{
                    distance = getDistance(robotPosition, Constants.Field.Reef.sideLocations[i]);
                }
                if(minDistance == -1){
                    minDistance = distance;
                }
                else if(distance < minDistance){
                    minDistance = distance;
                    minIndex = i;
                }
            }

            Pose2d targetPose = new Pose2d();
            if(leftSide){
                 targetPose = Constants.Field.Reef.leftLocations[minIndex];
            }
            else{
          
                 targetPose = Constants.Field.Reef.rightLocations[minIndex];
            }
            if(ally.get() == DriverStation.Alliance.Red){
                targetPose = pose2red(targetPose);
            }
          return targetPose;
        };
        return targetSupplier;
    }

    public static double getDistance(Pose2d pos1, Translation2d pos2){
        return Math.sqrt((pos1.getX() - pos2.getX())*(pos1.getX() - pos2.getX()) + (pos1.getY() - pos2.getY())*(pos1.getY() - pos2.getY()));
    }
    public static Translation2d transform2red(Translation2d transform1){
        Translation2d fieldCenter = new Translation2d(Constants.Field.fieldLength, Constants.Field.fieldWidth);
        double redXpos =  fieldCenter.getX()-transform1.getX();
        double redYpos =  fieldCenter.getY()-transform1.getY();
        return new Translation2d(redXpos, redYpos);
    }
    public static Pose2d pose2red(Pose2d pose1){
        Translation2d fieldCenter = new Translation2d( Constants.Field.fieldLength, Constants.Field.fieldWidth);
        double redXpos =  fieldCenter.getX()-pose1.getX();
        double redYpos =  fieldCenter.getY()-pose1.getY();
        double redTheta = pose1.getRotation().getRadians() + Math.PI;
        return new Pose2d(redXpos, redYpos, new Rotation2d(redTheta));
    }
}
