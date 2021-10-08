package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name = "meet1AutoV2")


public class meet1AutoV2 extends LinearOpMode
{


    @Override
    public void runOpMode() throws InterruptedException
    {
        double loopcount = 0;
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //ElapsedTime whatever = new ElapsedTime();

        //robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        waitForStart();



        //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        encoderDrive(400, .75);

        //turn(-90, .75);

        //encoderDrive(1400, .75);

        stopDrive();

       while(opModeIsActive()){
           telemetry.addData("loopcount", loopcount);
           telemetry.update();
           loopcount++;
       }
    }


    public void encoderDrive (int distance, double speed) {
        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.motorLF.setTargetPosition(distance * 3/4 + robot.motorLF.getCurrentPosition());
        robot.motorLM.setTargetPosition(distance + robot.motorLM.getCurrentPosition());
        robot.motorLB.setTargetPosition(distance * 3/4 + robot.motorLB.getCurrentPosition());
        robot.motorRF.setTargetPosition(distance * 3/4 + robot.motorRF.getCurrentPosition());
        robot.motorRM.setTargetPosition(distance + robot.motorRM.getCurrentPosition());
        robot.motorRB.setTargetPosition(distance * 3/4 + robot.motorRB.getCurrentPosition());

        robot.motorLF.setPower(speed * 3/4);
        robot.motorLM.setPower(speed);
        robot.motorLB.setPower(speed * 3/4);
        robot.motorRF.setPower(speed * 3/4);
        robot.motorRM.setPower(speed);
        robot.motorRB.setPower(speed * 3/4);

        robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.motorRF.isBusy() && opModeIsActive())
        {
            telemetry.addData("LF encoder value", robot.motorLF.getCurrentPosition());
            telemetry.addData("LM encoder value", robot.motorLM.getCurrentPosition());
            telemetry.addData("LB encoder value", robot.motorLB.getCurrentPosition());
            telemetry.addData("RF encoder value", robot.motorRF.getCurrentPosition());
            telemetry.addData("RM encoder value", robot.motorRM.getCurrentPosition());
            telemetry.addData("RB encoder value", robot.motorRB.getCurrentPosition());
            telemetry.update();
        }


    }

    public void stopDrive () {
        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.motorLB.setPower(0);
        robot.motorLM.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorRB.setPower(0);
        robot.motorRM.setPower(0);
        robot.motorRF.setPower(0);

        telemetry.addData("im here", 0);
        telemetry.update();
    }

    public void turn (double angle, double speed) {
        RobotHardware robot = new RobotHardware(hardwareMap);

       // robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

      // while (robot.angles.firstAngle > angle && opModeIsActive())
      // {
      //     //robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

      //     robot.motorLB.setPower(speed * 3 / 4);
      //     robot.motorLM.setPower(speed);
      //     robot.motorLF.setPower(speed * 3 / 4);

      //     robot.motorRB.setPower(-speed * 3 / 4);
      //     robot.motorRM.setPower(-speed);
      //     robot.motorRF.setPower(-speed * 3 / 4);

      //     telemetry.addData("heading", robot.angles.firstAngle);
      //     telemetry.update();
      // }
    }
}
