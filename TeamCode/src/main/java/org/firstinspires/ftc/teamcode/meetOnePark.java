package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="meetOnePark")
//@Disabled


public class meetOnePark extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();


        RobotHardware robot = new RobotHardware(hardwareMap);

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ElapsedTime whatever = new ElapsedTime();

        waitForStart();

        forward(1600,.9);

        sleep(2500);

        pivot(800,0.75);

        sleep(2000);

        forward(1000,.2);

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
