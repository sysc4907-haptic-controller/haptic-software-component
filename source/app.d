import std.stdio : write, writeln, readln, stderr, stdout;
import std.string : fromStringz;
import std.datetime : seconds;
import std.variant : Variant;
import std.concurrency : spawn;
import std.file;
import std.exception : enforce;
import std.conv : to;
import std.format;
import std.math;
import std.math.rounding : round;
import kaleidic.lubeck : mldivide, inv;
import mir.ndslice;
import std.datetime.stopwatch : StopWatch, AutoStart;
import bindbc.sdl;
import bindbcLoader = bindbc.loader.sharedlib;
import serialport : SerialPort;
import core.time : msecs, usecs;

import serial : serialReceiveWorker;
import sim;
import pid;
import sensorvalueholder;

// Motor-related Values
private double noLoadRpm = 7600;
private double motorPeakVoltage = 30;
double motorTorqueConstant = 248.1609513;

//TODO: GET THE RIGHT COORDS
//units: PIXELS
const double Y_BOTTOM_LINKAGE = 1584;
const double X_LEFT_ENCODER = 465;
const double X_RIGHT_ENCODER = 915;

//units: PIXELS
const double LEN_BOTTOM_ARMS = 700;
const double LEN_TOP_ARMS = 900;
const double LEN_BETWEEN_SHAFTS = 450;

class InvalidAnglesException : Exception
{
    this(string msg) @safe pure nothrow @nogc
    {
        super(msg);
    }
}

struct ValuesFromInitialAngles
{
    PositionVector pos;
    double theta3;
    double theta4;
}

auto calculateRequiredCurrent(double sensorReading)
{
    float sensitivity = 100.0 / 500.0;
    float Vref = 2500;
    float voltage = 4.88 * sensorReading;

    return (voltage - Vref) * sensitivity;
}

auto inverseTransposeJacobian(double theta1, double theta2, double theta3, double theta4)
{
    auto jacobian = slice!double(2, 2);
    auto row0 = jacobian[0];
    auto row1 = jacobian[1];
    //This is the transpose Jacobian
    row0[0] = (LEN_BOTTOM_ARMS * sin(theta1 - theta3) * sin(theta4)) / (sin(theta3 - theta4));
    row1[0] = (LEN_BOTTOM_ARMS * sin(theta4 - theta2) * sin(theta3)) / (sin(theta3 - theta4));
    row0[1] = -(LEN_BOTTOM_ARMS * sin(theta1 - theta3) * cos(theta4)) / (sin(theta3 - theta4));
    row1[1] = -(LEN_BOTTOM_ARMS * sin(theta4 - theta2) * cos(theta3)) / (sin(theta3 - theta4));
    //returning the inverse of teh transpose jacobian
    return jacobian.inv;
}

ValuesFromInitialAngles getValuesFromInitialAngles(double theta1, double theta2,
        int prevY, VelocityVector v)
{
    double leftArmJointX = X_LEFT_ENCODER + LEN_BOTTOM_ARMS * cos(theta1);
    double rightArmJointX = X_RIGHT_ENCODER + LEN_BOTTOM_ARMS * cos(theta2);
    double leftArmJointY = Y_BOTTOM_LINKAGE - LEN_BOTTOM_ARMS * sin(theta1);
    double rightArmJointY = Y_BOTTOM_LINKAGE - LEN_BOTTOM_ARMS * sin(theta2);

    double v1 = (rightArmJointY - leftArmJointY) / (leftArmJointX - rightArmJointX);
    double v2 = (leftArmJointX * leftArmJointX + leftArmJointY * leftArmJointY
            - rightArmJointX * rightArmJointX - rightArmJointY * rightArmJointY) / (
            2 * (leftArmJointX - rightArmJointX));
    //double v2 = (-(LEN_BOTTOM_ARMS*LEN_BOTTOM_ARMS) + rightArmJointX*rightArmJointX + rightArmJointY*rightArmJointY) / (2*(rightArmJointX - leftArmJointX));
    double v3 = 1.0 + v1 * v1;
    double v4 = 2.0 * (v1 * v2 - v1 * leftArmJointX - leftArmJointY);

    double v5 = leftArmJointX * leftArmJointX - LEN_TOP_ARMS * LEN_TOP_ARMS
        + leftArmJointY * leftArmJointY - 2.0 * v2 * leftArmJointX + v2 * v2;

    //double v5 = LEN_BOTTOM_ARMS*LEN_BOTTOM_ARMS*LEN_TOP_ARMS*LEN_TOP_ARMS - 2.0*v2*leftArmJointX + v2*v2;

    //stderr.writeln("TEST:" ~ " | v1: " ~ to!string(v1) ~ " | v2: " ~ to!string(v2) ~ " | v3: " ~ to!string(v3) ~ " | v4: " ~ to!string(v4) ~ " | v5: " ~ to!string(v5));

    double det = v4 * v4 - 4 * v3 * v5;
    ValuesFromInitialAngles ret;

    if (det < 0)
    {
        if (det > -0.001)
        {
            det = 0;
        }
        else
        {
            throw new InvalidAnglesException("Impossible Angles Given");
        }
    }
    double endY = (-v4 - sqrt(det)) / (2.0 * v3);
    double endX = v1 * endY + v2;

    //Calculating theta3 and theta4 ADD PI
    double theta3 = atan(abs(endY - leftArmJointY) / abs(endX - leftArmJointX));
    if (endX < leftArmJointX)
    {
        theta3 = 2 * PI - theta3;
    }
    double theta4 = PI - atan(abs(endY - rightArmJointY) / abs(endX - rightArmJointX));
    if (endX > rightArmJointX)
    {
        theta4 = 2 * PI - theta4;
    }

    ret.pos = new PositionVector(cast(int) round(endX), cast(int) round(endY));
    ret.theta3 = theta3;
    ret.theta4 = theta4;
    return ret;
}

int main(string[] args)
{
    if (args.length < 2)
    {
        stderr.writeln("usage: project-monitor /path/to/port");
        return 1;
    }

    auto sdlLoadResult = loadSDL();

    if (sdlLoadResult != sdlSupport)
    {
        writeln("Failed to load SDL.");

        foreach (error; bindbcLoader.errors)
        {
            writeln("Loader error: ", fromStringz(error.error), ": ", fromStringz(error.message));
        }
        return 1;
    }

    auto sdlTTFLoadResult = loadSDLTTF();

    if (sdlTTFLoadResult != sdlTTFSupport)
    {
        writeln("Failed to load SDL TTF.");

        foreach (error; bindbcLoader.errors)
        {
            writeln("Loader error: ", fromStringz(error.error), ": ", fromStringz(error.message));
        }
        return 1;
    }

    auto sdlImageLoadResult = loadSDLImage();

    if(sdlImageLoadResult != sdlImageSupport) {

        writeln("Failed to load SDL Image.");

        foreach (error; bindbcLoader.errors)
        {
            writeln("Loader error: ", fromStringz(error.error), ": ", fromStringz(error.message));
        }
        return 1;
    }

    auto imgFlags = IMG_INIT_PNG | IMG_INIT_JPG;
    if (!(IMG_Init(imgFlags) != imgFlags))
    {
        writeln("SDL_image could not initialize! SDL_image Error: %s\n", IMG_GetError());
    }

    auto error = SDL_Init(SDL_INIT_VIDEO);

    // Create 1300x1000 window - size can be adjusted in future
    const auto windowWidth = 1300;
    const auto windowHeight = 1000;

    auto window = SDL_CreateWindow("2-DOF Hybrid Haptic Device",
            SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, windowWidth, windowHeight, 0);

    if (!window)
    {
        writeln("Could not create window: ", SDL_GetError());
        return 1;
    }

    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");

    auto renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    if (!renderer)
    {
        writeln("Could not create renderer: ", SDL_GetError());
        return 1;
    }

    scope (exit)
    {
        SDL_DestroyRenderer(renderer);
    }

    auto gravelSurface = IMG_Load("resources/gravel.jpg");
    auto gravelTexture = SDL_CreateTextureFromSurface(renderer, gravelSurface);

    auto riverSurface = IMG_Load("resources/river.jpg");
    auto riverTexture = SDL_CreateTextureFromSurface(renderer, riverSurface);

    shared serialport = new shared SerialPort(args[1], 115200);
    // TODO: See if we can get this to work
    // scope (exit)
    // {
    //     serialport.close();
    // }

    SensorValueHolder sensorValueHolder = new SensorValueHolder;
    spawn(&serialReceiveWorker, serialport, sensorValueHolder);

    auto black = SDL_Color(0, 0, 0);
    auto red = SDL_Color(255, 0, 0);
    auto blue = SDL_Color(0, 0, 255);
    auto orange = SDL_Color(255, 165, 0);
    auto grey = SDL_Color(128, 128, 128);
    // List of the elements a part of the system (walls, fields, surfaces)
    SimulationElement[] elements = [
        new GravelElement(600, 150, 400, 400, grey),
        new ViscousElement(200, 150, 400, 400, orange, 0.5),
        new MovementElement(200, 550, 800, 300, black)
    ];

    // Create the end effector representation in simulation
    int mouseX, mouseY;
    SDL_GetMouseState(&mouseX, &mouseY);
    EndEffector endEffector = new EndEffector(mouseX, mouseY);

    double THETA_1_INIT, THETA_2_INIT;

    THETA_2_INIT = acos((cast(double)(2 * LEN_TOP_ARMS - LEN_BETWEEN_SHAFTS)) / (2 * LEN_BOTTOM_ARMS));
    THETA_1_INIT = PI - THETA_2_INIT;

    auto background = SDL_Rect(0, 0, 1300, 1000);

    // Create visual representation of the end effector (green square)
    // auto cursor = SDL_Rect(endEffector.x, endEffector.y, 10, 10);

    //bool ledOn = false;
    bool mouseMode = false;

    Pid leftMotorController = new Pid(0);
    Pid rightMotorController = new Pid(0);

    bool initialize = true;
    bool printedError = false;

    auto sw = StopWatch(AutoStart.no);
    int gravelLoop = 0;
    ubyte[3] leftGravelArray = [200, 0x00, 200];
    ubyte[3] rightGravelArray = [200, 200, 0x00];

    // Event Loop
    while (true)
    {
        if (initialize)
        {
            byte[5] initializeMsg = [0x76, 0x76, 0x00, 0x00, 0x00];
            serialport.write(initializeMsg);
            initialize = false;
            sw.start();
        }

        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT || (event.type == SDL_KEYDOWN
                    && event.key.keysym.sym == SDLK_q))
            {
                return 0;
            }

            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_g)
            {
                //serialport.write([ledOn ? 1: 2]);
                mouseMode = !mouseMode;
            }
        }

        auto theta1 = THETA_1_INIT - sensorValueHolder.leftEncoder * (PI / 180.0) * (360.0 / 8192.0);
        auto theta2 = THETA_2_INIT - sensorValueHolder.rightEncoder * (PI / 180.0) * (360.0 / 8192.0);

        //writeln("THETAS: " ~to!string(theta1) ~ "," ~ to!string(theta2));

        // Clear render with a white background
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
        SDL_RenderFillRect(renderer, &background);
        SDL_RenderClear(renderer);

        // Draw each simulation element
       /* foreach (SimulationElement element; elements)
        {
            element.draw(renderer);

            // If a collision is detected between endeffector and a sim. element,
            //  adjust the cursor location such that it doesn't disconnect with the visual element
            if (endEffector.detectCollision(element) && mouseMode)
            {
                SDL_WarpMouseInWindow(window, endEffector.x, endEffector.y);
            }
        }*/
        elements[1].draw(renderer);
        SDL_RenderCopy(renderer, gravelTexture, null, &elements[0].rect);
        SDL_RenderCopy(renderer, riverTexture, null, &elements[2].rect);
        PositionVector endEffectorPosFromAngles;
        double theta3;
        double theta4;
        ValuesFromInitialAngles endPointsAndThetas;

        if (mouseMode)
        {
            // Update the end effector's data (position, time)
            SDL_GetMouseState(&mouseX, &mouseY);
            endEffector.update(mouseX, mouseY);
        }
        else
        {
            try
            {
                endPointsAndThetas = getValuesFromInitialAngles(theta1, theta2,
                        endEffector.y, endEffector.currVelocity);

                endEffectorPosFromAngles = endPointsAndThetas.pos;
                theta3 = endPointsAndThetas.theta3;
                theta4 = endPointsAndThetas.theta4;

                endEffector.update(endEffectorPosFromAngles);
                printedError = false;
            }
            catch (InvalidAnglesException e)
            {
                if (!printedError)
                {
                    stderr.writeln("ERROR CALCULATING END POINT FOR THE FOLLOWING THETA_1: " ~ to!string(
                            theta1) ~ " | THETA_2: " ~ to!string(theta2));
                    printedError = true;
                }
            }
            //writeln("End Effector POS: " ~to!string(endEffector.x) ~ "," ~ to!string(endEffector.y));
        }

        // Draw the cursor
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);
        endEffector.draw(renderer);

        // "Load" all elements to the render
        SDL_RenderPresent(renderer);

        if (!mouseMode)
        {
            // Define motor IDs
            enum ubyte rightMotorID = 1;
            enum ubyte leftMotorID = 2;
            enum ubyte rightBrakeID = 3;
            enum ubyte leftBrakeID = 4;
            bool mflag = false;
            bool bflag = false;
            writeln("t1: " ~ to!string(theta1) ~ " t2: " ~ to!string(theta2));
            //writeln("X: " ~ to!string(endEffector.x) ~ " Y: " ~ to!string(endEffector.y));
            //GRAVEL ELEM
            if (endEffector.x >= elements[0].rect.x
                    && endEffector.x <= elements[0].rect.x + elements[0].rect.w
                    && endEffector.y >= elements[0].rect.y
                    && endEffector.y <= elements[0].rect.y + elements[0].rect.h)
            {
                if (sw.peek() >= usecs(20 * 1000))
                {
                    ubyte leftPower = leftGravelArray[gravelLoop];
                    ubyte rightPower = rightGravelArray[gravelLoop];
                    gravelLoop = gravelLoop + 1 >= 3 ? 0 : gravelLoop + 1;

                    ubyte[7] leftBrake_msg = [
                        0x76, 0x76, 0x01, 0x3, leftBrakeID, leftPower, 0x00
                    ];
                    serialport.write(leftBrake_msg);

                    ubyte[7] rightBrake_msg = [
                        0x76, 0x76, 0x01, 0x3, rightBrakeID, rightPower, 0x00
                    ];

                    serialport.write(rightBrake_msg);
                    sw.reset();
                    writeln(rightBrake_msg);
                    stdout.flush();
                }
                bflag = true;
            }
            //HONEY ELEM
            else if (endEffector.x >= elements[1].rect.x
                    && endEffector.x <= elements[1].rect.x + elements[1].rect.w
                    && endEffector.y >= elements[1].rect.y
                    && endEffector.y <= elements[1].rect.y + elements[1].rect.h)
            {

                ubyte leftPower = 200;
                ubyte rightPower = 200;

                ubyte[7] leftBrake_msg = [
                    0x76, 0x76, 0x01, 0x3, leftBrakeID, leftPower, 0x00
                ];
                serialport.write(leftBrake_msg);

                ubyte[7] rightBrake_msg = [
                    0x76, 0x76, 0x01, 0x3, rightBrakeID, rightPower, 0x00
                ];
                serialport.write(rightBrake_msg);
                writeln(rightBrake_msg);
                stdout.flush();
                bflag = true;
            }else if (endEffector.x >= elements[2].rect.x
            && endEffector.x <= elements[2].rect.x + elements[2].rect.w
            && endEffector.y >= elements[2].rect.y
            && endEffector.y <= elements[2].rect.y + elements[2].rect.h)
            {
                    auto torques = mldivide(inverseTransposeJacobian(theta1, theta2, theta3, theta4), [0.02, 0.1].sliced(2,1));
                    //writeln("LEFT PERCENTAGE: " ~ to!string(torques[0][0]* 255/538.4));
                    //writeln("RIGHT PERCENTAGE: " ~ to!string(torques[1][0]* 255/538.4));
                    stdout.flush();
                    ubyte leftPower = to!ubyte(round(abs(torques[0][0] * 255/538.4)));
                    ubyte rightPower = to!ubyte(round(abs(torques[1][0] * 255/538.4)));
                    ubyte[7] leftMotor_msg = [
                        0x76, 0x76, 0x01, 0x3, leftMotorID, leftPower, ((torques[0][0] > 0) ? 0 : 1)
                    ];
                    serialport.write(leftMotor_msg);

                    ubyte[7] rightMotor_msg = [
                        0x76, 0x76, 0x01, 0x3, rightMotorID, rightPower, ((torques[1][0] > 0) ? 0 : 1)
                    ];
                    serialport.write(rightMotor_msg);
                    //writeln("RIGHT MOTOR: " ~ to!string(rightMotor_msg));
                    //writeln("LEFT MOTOR: " ~ to!string(leftMotor_msg));
                    mflag = true;
                    stdout.flush();
                }
            if(!bflag)
            {
                ubyte[7] leftBrake_msg = [
                    0x76, 0x76, 0x01, 0x3, leftBrakeID, 0, 0x00
                ];
                serialport.write(leftBrake_msg);

                ubyte[7] rightBrake_msg = [
                    0x76, 0x76, 0x01, 0x3, rightBrakeID, 0, 0x00
                ];
                serialport.write(rightBrake_msg);
            }
        if(!mflag) {
            ubyte[7] leftMotor_msg = [
                0x76, 0x76, 0x01, 0x3, leftMotorID, 0, 0x00
            ];
            serialport.write(leftMotor_msg);

            ubyte[7] rightMotor_msg = [
                0x76, 0x76, 0x01, 0x3, rightMotorID, 0, 0x00
            ];
            serialport.write(rightMotor_msg);
        }


            //writeln("StopWatch: " ~ to!string(sw.peek()));
        }

        //ubyte[7] leftMotor_msg = [0x76, 0x76, 0x01, 0x3, 4, 200, 0x00];
        //serialport.write(leftMotor_msg);
    }
}
