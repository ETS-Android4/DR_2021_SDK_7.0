package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class driveClass extends LinearOpMode
{
  
  //variables
  
  double pivot = 0;
  double dirX = 0;
  double dirY = 0;

    public DcMotorEx motorLF = null;
    public DcMotorEx motorLB = null;
    public DcMotorEx motorRF = null;
    public DcMotorEx motorRB = null;
  
  int quadrant;
  
  Orientation angles;
  
  public void drive(int moveAngle, double movePower, int turnAngle, double turnPower, DcMotorEx[] motors, BNO055IMU imu)
  {
      motorLF = hardwareMap.get(DcMotorEx.class, "motorLF");
      motorLB = hardwareMap.get(DcMotorEx.class, "motorLB");
      motorRF = hardwareMap.get(DcMotorEx.class, "motorRF");
      motorRB = hardwareMap.get(DcMotorEx.class, "motorRB");
      //insert drive code
      
      //angle wrap
      while (turnAngle > 180)
      {
          turnAngle -= 360;
      }
      while (turnAngle < -180)
      {
          turnAngle += 360;
      }
      
      
      //turn calculations
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      
      if (angles.firstAngle < turnAngle) 
      {
        pivot = -(((turnAngle - angles.firstAngle) /turnAngle)+ .2);
      }
      else if (angles.firstAngle > turnAngle)
      {
        pivot = (((turnAngle - angles.firstAngle) /turnAngle)+ .2);
      }
      
      //angle wrap
      while (moveAngle >= 360)
      {
          moveAngle -= 360;
      }
      while (moveAngle < 0)
      {
          moveAngle += 360;
      }
      
      //record quadrant
      if (moveAngle < 90) 
      {
        quadrant = 1;
      }
      else if (moveAngle < 180)
      {
        quadrant = 2;
      }
      else if (moveAngle < 270)
      {
        quadrant = 3;
      }
      else
      {
        quadrant = 4;
      }
      
      //90 degree angle wrap
      while (moveAngle > 90)
      {
          moveAngle -= 90;
      }
      while (moveAngle < 0)
      {
          moveAngle += 90;
      }
      
      //drive claculations
      dirY = Math.cos(moveAngle);
      dirX = Math.sin(moveAngle);
      
      if (quadrant == 1)
      {
        dirY = -dirY;
        dirX = -dirX;
      }
      else if (quadrant == 2)
      {
        dirY = dirY;
        dirX = -dirX;
      }
      else if (quadrant == 3)
      {
        dirY = dirY;
        dirX = dirX;
      }
      else
      {
        dirY = -dirY;
        dirX = dirX;
      }
      
     // drive set powers (velocities)
      motors[0].setPower(((-dirY + dirX) * movePower) + (pivot * turnPower));
      motors[1].setPower((-(dirX + dirY) * movePower) + (pivot * turnPower));
      motors[2].setPower(((-dirY - dirX) * movePower) - (pivot * turnPower));
      motors[3].setPower(((dirX -  dirY) * movePower) - (pivot * turnPower));
      
      
    }


    @Override
    public void runOpMode() throws InterruptedException
    {

    }
}
