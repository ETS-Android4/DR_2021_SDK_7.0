package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="motorCurrentTest")
//@Disabled

public class motorCurrentTest extends LinearOpMode
{

    public DcMotorEx intake1 = null;
    public DcMotorEx intake2 = null;
    
    double max1 = 0;
    double max2 = 0;
    
    @Override
    public void runOpMode() throws InterruptedException
    {
        
        intake1 = hardwareMap.dcMotorEx.get("intake1");
        intake2 = hardwareMap.dcMotorEx.get("intake2");


        waitForStart();
        
        intake1.setPower(1);
        intake2.setPower(1);

        while (opModeIsActive())
        {
            if(intake1.getCurrent() > max1) {
                max1 = intake1.getCurrent();
            }

            if(gamepad1.a) {
                max1 = 0;
            }
            
            if(intake2.getCurrent() > max2) {
                max2 = intake2.getCurrent();
            }

            if(gamepad1.a) {
                max2 = 0;
            }

            
            telemetry.addData("max Current intake1", intake1.getCurrent());
            telemetry.addData("max Current intake2", intake2.getCurrent());

            telemetry.update();

        }
    }
}
