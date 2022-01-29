package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="meetOneBlueDucks")
@Disabled


public class meetOneBlueDucks extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
       // FtcDashboard dashboard = FtcDashboard.getInstance();
       // telemetry = dashboard.getTelemetry();

       double loopCount = 0;

        RobotHardware robot = new RobotHardware(hardwareMap);

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ElapsedTime whatever = new ElapsedTime();

        waitForStart();

        pivot(200,.5);

        telemetry.addData("1",loopCount);
        telemetry.update();

        sleep();

        forward(-1500,-.5);

        telemetry.addData("2",loopCount);
        telemetry.update();

        sleep();


        robot.INservo1.setPower(-.7);
        robot.INservo2.setPower(-.7);


        telemetry.addData("",loopCount);
        telemetry.update();

        sleep();
        sleep();
        sleep();

        telemetry.addData("hshshshshshshhssh",loopCount);
        telemetry.update();

        loopCount = 1;
        robot.INservo1.setPower(0);
        robot.INservo2.setPower(0);


        sleep();
        sleep();

        pivot(550, .5);

        sleep();

        forward(-1700,-.5);

        sleep();
        sleep();

        pivot(50, .5);

        sleep();
        sleep();
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
