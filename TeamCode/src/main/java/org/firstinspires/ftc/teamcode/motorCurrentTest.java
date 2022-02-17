package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name="motorCurrentTest")
//@Disabled

public class motorCurrentTest extends LinearOpMode
{

    public DcMotorEx intake1 = null;
    public DcMotorEx intake2 = null;
    
    double max1 = 0;
    double max2 = 0;
    double current1;
    double current2;
    
    @Override
    public void runOpMode() throws InterruptedException
    {
        
        intake1 = (DcMotorEx) hardwareMap.dcMotor.get("intake1");
        intake2 = (DcMotorEx) hardwareMap.dcMotor.get("intake2");


        waitForStart();
        
        intake1.setPower(-1);
        intake2.setPower(-1);

        while (opModeIsActive())
        {
            current1 = intake1.getCurrent(CurrentUnit.AMPS);
            if(current1 > max1) {
                max1 = current1;
            }

            if(gamepad1.a) {
                max1 = 0;
            }

            current2 = intake2.getCurrent(CurrentUnit.AMPS);
            if(current2 > max2) {
                max2 = current2;
            }

            if(gamepad1.a) {
                max2 = 0;
            }

            
            telemetry.addData("max Current intake1", max1);
            telemetry.addData("max Current intake2", max2);

            telemetry.update();

        }
    }
}
