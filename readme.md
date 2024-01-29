![](src/main/java/frc/robot/lib/MythDigLogo1.0.svg)

## Overview
This code uses the CTRE swerve module code. The code has been integrated into the command based programming framework.
The development of this code was done by using the Phoenix Pro Examples and not the Tuner generator. Changes from the base code are:
- Removed telemetry of mechanisms2d
- uses 3 swerve modules instead of 4
- Rearanged code to fit a better Command Based framework
- Created DriveDefaultCommand to call different drive modes based on user inputs.
- Added subsystem interface for updating dashboard
- Added autonomous getter from dashboard
- Removed all path planner logic


## Robot 
The robot used for this code is our typical 3 wheel swerve drive using Swerve Xs from WCProducts.net. All motors are Falcon FXs with CANCoders.
A CANivore is used with a PRO license to get FOC out of the motors.

## Change for your use
To use this code you must understand the code and adjust all changes for your system. 
Changes needed that I understand at this point are:
- Add another motor for typical 4 wheel swerve. We are one of the only foolish teams to do 3 wheel swerve.
- Change your drive base length and width.
- Tune your PID variables.
- Change all your CANIDs.
- Change gear ratios
- Debug what I have not tested yet...

## Subsystems
- **Drivetrain**
  The drivetrain is our typical swerve drive system that utilizes WCProduts.net swerve X modules. Our drivetrain utilized 3 modules instead of the typical 3.
  The chassis, or frame, this year is a new approach utilizing 3 inch angle aluminum that is 3/16 inch thick. This is a high speed game and impacts are expected.
  The choosen frame strucutre will not bend like our typical 2040 structure that has bent before. 
- **Intake**
  The intake is planned to be a rotating wheeled intake that rotates out over the bumpers picks up Notes from the floor only.
  After picking up the note from the floor the Note is immediatley deliverd to the shooter or the Tramper.
- **Shooter**
  The shooter is a wheeled mechanism that launches the Note at varied angles to allow the robot to score from different distances
- **Climber**
  The climber is a two hook solution that raises the robot of the ground high enough for the Tramper to score in the Trap.
- **Tramper**
  The tramper is responsible for getting a game piece from the source area of from the intake. It is utilized to score in the Amp or Trap.

## Vision - Camera
The vision strategy promoted in the CTRE code is not being used in this code. If we have a chance we will bring it back in. 


## Autonomous
- Our basic drive strategy is to drive at a angle and velocity in Angle based field centric mode. This allows us to drive in any angle and have the robot PID to an angle. This prevents us from using a PID loop to drive to distance. 

## Pull/Push Requests
Please add a Pull Request for changes that you would like to see.
If you have code fixes or addition you want in, please make a push request with the changes.
More people working on code makes it better if we coordinate and agree on changes/additions.
There are lots of ways to solve a problem when coding, so please understand if changes are not accepted.

## Command Based Programming
The Command based approach used for this code is to use a default command that is associated with the subsystem. The schedular will call the execute() method in the command and from there the subsystem methods are called to drive the motors or whatever you want. 

The other approach for this is where you control all your motors in the subsystem perodic() methods. Commands use the execute method to set variables in the subsystem which the subsystem periodic will act on.

Many ways to do programming, we are accustomed to using the first approach because it seems a little easier to manage but may have more commands sometimes. Pick your approach and deal with it.
