package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="DuckAutoBlueV2")
@Disabled

public class DuckAutoBlueV2 extends LinearOpMode
{

//hardware map stuff

    @Override
    public void runOpMode() throws InterruptedException
    {

        RobotHardware robot = new RobotHardware(hardwareMap);
        
        //hardware map stuff
        
        //camera code stuff

        waitForStart();

        while (opModeIsActive())
        {
            drive.timeDrive(500, 1, STRAFE_RIGHT, motors);
            
            drive.OrientationDrive(90, 1, motors, imu);
            
            drive.timeDrive(500, 1, STRAFE_LEFT, motors);
            
            drive.timeDrive(500, 1, FORWARD_LEFT, motors);
            
            //spin carosel
            
            sleep(5000);
            
            //stop carosel
            
            drive.encoderDrive(1500, BACKWARD, 1, motors);
            
            drive.timeDrive(500, 1, STRAFE_RIGHT, motors);
            
            drive.OrientationDrive(180, 1, motors, imu);
            
            //prime to raise arm
            
            drive.timeDrive(500, 1, FORWARD, motors);
            
            //raise arm
            
            drive.encoderDrive(2000, BACKWARD, 1, motors);
            
            //place block
            
            sleep(1000);
            
            //return servo
            
            drive.timeDrive(500, 1, FORWARD, motors);
            
            //lower lift
            
            drive.OrientationDrive(150, 1, motors, imu);
            
            drive.timeDrive(3000, 1, FORWARD, motors);
        }
    }
