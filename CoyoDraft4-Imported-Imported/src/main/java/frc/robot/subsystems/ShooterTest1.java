// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterTest1 extends SubsystemBase
{
    //Declare our motors here first
    static CANSparkMax pivotMotor1 = new CANSparkMax(35,MotorType.kBrushless);
    static CANSparkMax pivotMotor2 = new CANSparkMax(36, MotorType.kBrushless);
    static CANSparkMax shooter1 = new CANSparkMax(37,MotorType.kBrushless);
    static CANSparkMax shooter2 = new CANSparkMax(38,MotorType.kBrushless);
    static CANSparkMax intake = new CANSparkMax(39,MotorType.kBrushless);
    //intakeEncoder = intake.getAbsoluteEncoder();


    static boolean reset = false;
    //0.125
    //0.75
    static PIDController PID = new PIDController(2.25,0.075,0.5);
    static boolean xButtonPressed = false;
    static boolean yButtonPressed = false;
    static boolean aButtonPressed = false;
    
//I made these changes (adi)
     //private final AbsoluteEncoder throughBore;
    // throughBore = pivotMotor2.getAbsoluteEncoder(Type.kDutyCycle);


    //I'm adding this (adi)
    static boolean LeftBumperPressed = false;
    static boolean PID4 = false;
    static boolean bButtonPressed = false;
    //
    static boolean PID1 = false;
    static boolean PID2 = false;
    static boolean PID3 = false;

    static double expectedAngle = 0.0;

    //Limelight Stuff

    //Table of values                                        
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    static NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    static NetworkTableEntry tid = table.getEntry("tid");

    //read values periodically
    public double x = tx.getDouble(0.0);
    public double y = ty.getDouble(0.0);
    public double area = ta.getDouble(0.0);

    //Constants
    static double kSpeakerHeightInches = 78;
    static double kSpeakerTagHeightInches = 57;
    static double kBotHeightInches = 9;
    static double kLimelightAngle = 31;
    static double kSpeakerShotMaxRadius = 150;
    static double kSpeakerShotMinRadius = 0;

//This actually works!

    private static SparkAbsoluteEncoder throughBore = pivotMotor2.getAbsoluteEncoder();  


      //Limelight Updates
  //This method is always running no matter what
public static void SpeakerAllignment()
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



    public static void shoot(double RightTrigger, double getAxis, double LeftTrigger,
    boolean xButtonPressed,boolean LeftBumperPressed,boolean yButtonPressed,boolean aButtonPressed,boolean bButtonPressed)
    {
     
        PID.setTolerance(0.05);
        SmartDashboard.putNumber("Encoder Data",throughBore.getPosition());
       // System.out.println("new encoder value: " + throughBore2.getPosition());
        pivotMotor2.follow(pivotMotor1,true);
        if(PID3)
        {
            if(Math.abs(throughBore.getPosition()-0.0912)<0.05)
            {
                PID3 = false;
            }  
            else
            {
                SmartDashboard.putNumber("Voltage",PID.calculate(throughBore.getPosition(),0.0912));
                pivotMotor1.set(PID.calculate(throughBore.getPosition(),0.0912));
            } 
        }
        else if(PID2)
        {
             if(Math.abs(throughBore.getPosition()-expectedAngle)<0.05)
            {
                PID2 = false;
            }  
            else
            {
                SmartDashboard.putNumber("Voltage",PID.calculate(throughBore.getPosition(),expectedAngle));
                pivotMotor1.set(PID.calculate(throughBore.getPosition(),expectedAngle));
            } 
        }
        else if(PID1)
        {
            if(Math.abs(throughBore.getPosition()-0.333)<0.05)
            {
                PID1 = false;
            }  
            else
            {
                SmartDashboard.putNumber("Voltage",PID.calculate(throughBore.getPosition(),0.333));
                pivotMotor1.set(PID.calculate(throughBore.getPosition(),0.333));
            } 
        }
       //shooter PID at "0.52" (added by adi)
        else if (PID4)
        {
            if(Math.abs(throughBore.getPosition()-0.655)<0.05)
            {
                PID4 = false;
            }  
            else
            {
                SmartDashboard.putNumber("Voltage",PID.calculate(throughBore.getPosition(),0.655));
                pivotMotor1.set(PID.calculate(throughBore.getPosition(),0.655));
            } 
        }
        else
        {
            if(xButtonPressed)
            {
                PID1 = true;
            }
            
            //added by ADI 
            if(bButtonPressed)
            {
                PID4 = true;
            }
            if(RightTrigger == 1)
            {
                shooter2.follow(shooter1, true);
                shooter1.setVoltage(11);
                //shooter1.setInverted(!shooter2.getInverted());
            }

            if (RightTrigger == 0)
            {
                shooter1.stopMotor();
                shooter2.stopMotor();
            }

            if(LeftTrigger == 1)
            {
                intake.setInverted(true);
                intake.set(2024); //direction is good!
            }
            if(LeftTrigger == 0)
            {
                
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



            //E stop
            if(yButtonPressed)
            {
                //PID2 = true;
                SpeakerAllignment();
            }

            if(aButtonPressed)
            {
                PID3 = true;
            }
        }
    }

    public static void stopPivot()
    {
        pivotMotor1.stopMotor();
        pivotMotor2.stopMotor();
    }

    public static void pivot(boolean upPivot, boolean downPivot)
    {
        if(throughBore.getPosition()<=-900)
        {
           // pivotMotor1.stopMotor();
            //pivotMotor2.stopMotor();
        }
        else{
        pivotMotor2.follow(pivotMotor1,true);
       
        if(upPivot)
        {
            SmartDashboard.putBoolean("Voltage ",true);
            pivotMotor1.setVoltage(2.5);
        }
        else if (downPivot) 
        {
            SmartDashboard.putBoolean("Voltage ",true);
            pivotMotor1.setVoltage(-2);
        }
         }
    }

}




