package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class drivClass extends LinearOpMode
{
  
  //variables
  
  double pivot = 0;
  double dirX = 0;
  double dirY = 0;
  
  int quadrant;
  
  public void drive(int moveAngle, double movePower, int turnAngle, double turnPower, DcMotorEx[] motors) 
  {
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
      dirY = Math.cosine(moveAngle);
      dirX = Math.sine(moveAngle);
      
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
      
      //drive set powers (velocities)
      motors[0] /* .set velocity */ (((-dirY + dirX) * movePower) + (pivot * turnPower)) * 2987;
      motors[1] /* .set velocity */ ((-(dirX + dirY) * movePower) + (pivot * turnPower)) * 2987;
      motors[2] /* .set velocity */ (((-dirY - dirX) * movePower) - (pivot * turnPower)) * 2987;
      motors[3] /* .set velocity */ (((dirX -  dirY) * movePower) - (pivot * turnPower)) * 2987;
      
      
    }
}
