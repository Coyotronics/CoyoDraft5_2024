// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterTest1 extends SubsystemBase
{
    //Declare our motors here first
    static CANSparkMax pivotMotor1 = new CANSparkMax(35,MotorType.kBrushless);
    static CANSparkMax pivotMotor2 = new CANSparkMax(36, MotorType.kBrushless);
    CANSparkMax shooter1 = new CANSparkMax(37,MotorType.kBrushless);
    CANSparkMax shooter2 = new CANSparkMax(38,MotorType.kBrushless);
    CANSparkMax intake = new CANSparkMax(39,MotorType.kBrushless);
    static boolean reset = false;
    PIDController PID = new PIDController(6.5,0.13,0.5);
    static boolean xButtonPressed = false;
    static boolean yButtonPressed = false;
    static boolean bButtonPressed = false;
    static boolean PID1 = false;
    static boolean PID2 = false;
    static boolean PID3 = false;
    static double expectedAngle = 0.0;

    //Limelight Stuff

    //Table of values                                        
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tid = table.getEntry("tid");

    //read values periodically
    public double x = tx.getDouble(0.0);
    public double y = ty.getDouble(0.0);
    public double area = ta.getDouble(0.0);

    //Constants
    double kSpeakerHeightInches = 78;
    double kSpeakerTagHeightInches = 57;
    double kBotHeightInches = 9;
    double kLimelightAngle = 31;
    double kSpeakerShotMaxRadius = 150;
    double kSpeakerShotMinRadius = 0;

    private static DutyCycleEncoder throughBore = new DutyCycleEncoder(0);

    
 

  //Limelight Updates
  //This method is always running no matter what
public void SpeakerAllignment()
{
    long currentId = tid.getInteger(-1);
    SmartDashboard.putNumber("Current ID",currentId);
    
    //Read Id first
    //For testing purposes I have it set to just one tag but later i'll add the tags from both sides
    if(currentId==7||currentId==8)
    {
        
        //Read Vertical Offset Value
        double targetOffset = ty.getDouble(0.0);

        //Convert total angle to Radians
        double totalAngleRads = (kLimelightAngle+targetOffset)*(3.14159/180);

        //Solve for distance using formula
        double distance = (kSpeakerTagHeightInches-kBotHeightInches)/Math.tan(totalAngleRads);

        //This puts the distance into ShuffleBoard so that we can see if the calculations are correct
        //TEST WITH THIS FIRST BY COMMENTING OUT THE REST OF THE CODE AND MAKE SURE THE DISTANCE IS CORRECT
        SmartDashboard.putNumber("Current Distance",distance);

        //Check if distance is in range
        if(distance<kSpeakerShotMaxRadius&&distance>kSpeakerShotMinRadius)
        {
            //Send Green LEDs
            
            //Needs to begin adjusting pivot
            double expectedTurning = Math.atan(kSpeakerHeightInches/distance);
            double expectedTurningDegrees = Math.toDegrees(expectedTurning);
            SmartDashboard.putNumber("Expected Turning",(expectedTurningDegrees/180.0)+0.314);
            expectedAngle = (expectedTurningDegrees/180.0)+0.314;
            //pivoting(throughBore.getDistance(),expectedTurningDegrees);
        }
        else
        {
            //Send Red LEDs
        }
    }
}

public void resetPosition()
{
    //Set back to 0
    throughBore.reset();    
}


    public void shoot(double getRightTrigger, double getAxis, boolean getAButtonPressed, boolean getAButtonReleased , 
    boolean xButtonPressed,double getLeftTrigger,boolean yButtonPressed,boolean bButtonPressed)
    {
     
        PID.setTolerance(0.05);
        SmartDashboard.putNumber("Encoder Data NEW VALUE",throughBore.get());
        System.out.println("encoder value: " + throughBore.get());
        pivotMotor2.follow(pivotMotor1,true);
        if(PID3)
        {
            if(Math.abs(throughBore.get()-0.1)<0.05)
            {
                PID3 = false;
            }  
            else
            {
                SmartDashboard.putNumber("Voltage",PID.calculate(throughBore.get(),0.1));
                pivotMotor1.set(PID.calculate(throughBore.get(),0.1));
            } 
        }
        else if(PID2)
        {
             if(Math.abs(throughBore.get()-expectedAngle)<0.05)
            {
                PID2 = false;
            }  
            else
            {
                SmartDashboard.putNumber("Voltage",PID.calculate(throughBore.get(),expectedAngle));
                pivotMotor1.set(PID.calculate(throughBore.get(),expectedAngle));
            } 
        }
        else if(PID1)
        {
           
            if(Math.abs(throughBore.get()-0.75)<0.05)
            {
                PID1 = false;
            }  
            else
            {
                SmartDashboard.putNumber("Voltage",PID.calculate(throughBore.get(),0.75));
                pivotMotor1.set(PID.calculate(throughBore.get(),0.75));
            } 
        }
        else
        {
            if(xButtonPressed)
            {
                PID1 = true;
            }
            if(getRightTrigger == 1)
            {
                shooter2.follow(shooter1, true);
                shooter1.setVoltage(11);
                //shooter1.setInverted(!shooter2.getInverted());
            }

            if (getRightTrigger == 0)
            {
                shooter1.stopMotor();
                shooter2.stopMotor();
            }


            if(getAButtonPressed)
            {
                intake.setInverted(true);
                intake.set(4000); //direction is good!
            }
            if(getAButtonReleased)
            {
                intake.set(-400);
                intake.stopMotor();
            }

            if(getAxis < -0.1)
            {
                pivot(true, false);
            }
            if(getAxis > 0.1)
            {
                pivot(false, true);
            }
            if(getAxis > -0.05 && getAxis < 0.05)
            {
                stopPivot();
            }

            //Method Rese
            if(getLeftTrigger==1)
            {
                resetPosition();
            }

            //E stop
            if(yButtonPressed)
            {
                PID2 = true;
                SpeakerAllignment();
            }

            if(bButtonPressed)
            {
                PID3 = true;
            }
        }
    }

    public void stopPivot()
    {
        pivotMotor1.stopMotor();
        pivotMotor2.stopMotor();
    }

    public void pivot(boolean upPivot, boolean downPivot)
    {
        if(throughBore.getDistance()<=-900)
        {
           // pivotMotor1.stopMotor();
            //pivotMotor2.stopMotor();
        }
        else{
        pivotMotor2.follow(pivotMotor1,true);
       
        if(upPivot)
        {
            SmartDashboard.putBoolean("Voltage ",true);
            pivotMotor1.setVoltage(1.5);
        }
        else if (downPivot) 
        {
            SmartDashboard.putBoolean("Voltage ",true);
            pivotMotor1.setVoltage(-1.5);
        }
         }
    }

}





