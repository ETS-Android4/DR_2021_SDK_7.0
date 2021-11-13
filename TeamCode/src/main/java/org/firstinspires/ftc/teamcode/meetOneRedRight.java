package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="meetOneRed")
//@Disabled


public class meetOneRedRight extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

       int loopCount = 0;

        RobotHardware robot = new RobotHardware(hardwareMap);

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ElapsedTime whatever = new ElapsedTime();

        waitForStart();

            forward(800,.9);

        robot.arm.setTargetPosition(650);

        robot.arm.setPower(0.75);

        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(300);

        robot.servo2.setPosition(.33);

        telemetry.addData("gate one opened",loopCount);
        telemetry.update();

            sleep(2000);

           // pivot(1000, -1000, .5, -.5);


        robot.servo.setPosition(1);

        telemetry.addData("gate two opened", loopCount);
        telemetry.update();

        sleep(2000);

        robot.servo2.setPosition(.75);

        robot.arm.setTargetPosition(-400);

        robot.arm.setPower(-0.25);

        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("gate three opened", loopCount);
        telemetry.update();

        sleep();

        forward(100,.7);

        sleep();

        pivot(-950,-.5);

        sleep(2500);

        forward(8000,1);

        sleep(3500);




       //robot.servo3.setPosition(.25);



//            while(loopCount < 500)
//            {
//                robot.INservo1.setPower(.5);
//                robot.INservo2.setPower(.5);
//                loopCount++;
//            }
//
//            robot.INservo1.setPower(0);
//            robot.INservo2.setPower(0);
//            loopCount = 0;

           // forward(2000,.7);





        }


    void forward (int distance, double power) {
        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.motorRF.setTargetPosition(distance + robot.motorRF.getCurrentPosition());
        robot.motorRM.setTargetPosition(distance + robot.motorRM.getCurrentPosition());
        robot.motorRB.setTargetPosition(distance + robot.motorRB.getCurrentPosition());
        robot.motorLM.setTargetPosition(distance + robot.motorLM.getCurrentPosition());
        robot.motorLF.setTargetPosition(distance + robot.motorLF.getCurrentPosition());
        robot.motorLB.setTargetPosition(distance + robot.motorLB.getCurrentPosition());

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

    void pivot (int distance, double power) {
        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.motorRF.setTargetPosition(distance + robot.motorRF.getCurrentPosition());
        robot.motorRM.setTargetPosition(distance + robot.motorRM.getCurrentPosition());
        robot.motorRB.setTargetPosition(distance + robot.motorRB.getCurrentPosition());
        robot.motorLM.setTargetPosition(-distance + robot.motorLM.getCurrentPosition());
        robot.motorLF.setTargetPosition(-distance + robot.motorLF.getCurrentPosition());
        robot.motorLB.setTargetPosition(-distance + robot.motorLB.getCurrentPosition());

        robot.motorRF.setPower(power * .75);
        robot.motorRM.setPower(power);
        robot.motorRB.setPower(power * .75);
        robot.motorLB.setPower(power * -.75);
        robot.motorLM.setPower(power * -1);
        robot.motorLF.setPower(power * -.75);

        robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    void sleep (){
        sleep(1000);
    }
}
//robot.wobble.setTargetPosition(upPosition);
//                robot.wobble.setPower(0.7);
//                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
