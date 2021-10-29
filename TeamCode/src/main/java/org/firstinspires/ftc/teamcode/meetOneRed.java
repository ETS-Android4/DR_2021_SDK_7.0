package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="meetOneRed")
//@Disabled


public class meetOneRed extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        int loopCount = 0;

        RobotHardware robot = new RobotHardware(hardwareMap);

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ElapsedTime whatever = new ElapsedTime();

        robot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

            forward(-1000,-.5);

          //  while(loopCount < 500)
          //  {
          //      robot.INservo1.setPower(.5);
          //      robot.INservo2.setPower(.5);
          //      loopCount++;

          //  }

           // robot.INservo1.setPower(0);
           // robot.INservo2.setPower(0);
           // loopCount = 0;

           // forward(2000,.7);





        }


    void forward (int distance, double power) {
        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.motorLB.setTargetPosition(distance);
        robot.motorRF.setTargetPosition(distance);
        robot.motorRM.setTargetPosition(distance);
        robot.motorRB.setTargetPosition(distance);
        robot.motorLM.setTargetPosition(distance);
        robot.motorLF.setTargetPosition(distance);

        robot.motorRF.setPower(power * .75);
        robot.motorRM.setPower(power);
        robot.motorRB.setPower(power * .75);
        robot.motorLB.setPower(power * .75);
        robot.motorLM.setPower(power);
        robot.motorLF.setPower(power * .75);

        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    void turn (int distanceL, int distanceR, double powerL, double powerR) {
        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.motorLB.setTargetPosition(distanceL);
        robot.motorRF.setTargetPosition(distanceR);
        robot.motorRM.setTargetPosition(distanceR);
        robot.motorRB.setTargetPosition(distanceR);
        robot.motorLM.setTargetPosition(distanceL);
        robot.motorLF.setTargetPosition(distanceL);

        robot.motorRF.setPower(powerR * .75);
        robot.motorRM.setPower(powerR);
        robot.motorRB.setPower(powerR * .75);
        robot.motorLB.setPower(powerL * .75);
        robot.motorLM.setPower(powerL);
        robot.motorLF.setPower(powerL * .75);

        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    void sleep (){
        sleep(10000);
    }
}
//robot.wobble.setTargetPosition(upPosition);
//                robot.wobble.setPower(0.7);
//                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
