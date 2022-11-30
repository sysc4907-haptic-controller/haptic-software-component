import std.stdio : write, writeln, readln, stderr;
import std.string : fromStringz;
import std.datetime : seconds;
import std.variant : Variant;
import std.concurrency : receiveTimeout, receive, spawn, Tid, thisTid, send;
import std.file;
import std.conv : to;
import std.format;

import bindbc.sdl;
import bindbcLoader = bindbc.loader.sharedlib;
import serialport : SerialPort;

import serial : serialReceiveWorker, SerialMessage;
import sim;

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
    scope (exit)
    {
        SDL_DestroyRenderer(renderer);
    }

    if (!renderer)
    {
        writeln("Could not create renderer: ", SDL_GetError());
        return 1;
    }

    shared serialport = new shared SerialPort(args[1], 115200);
    // TODO: See if we can get this to work
    // scope (exit)
    // {
    //     serialport.close();
    // }

    error = TTF_Init();

    if (error != 0)
    {
        writeln("Could not initialize SDL_ttf: ", TTF_GetError());
        return 1;
    }

    auto dejaVuSans = TTF_OpenFont("DejaVuSans.ttf", 48);
    scope (exit)
    {
        TTF_CloseFont(dejaVuSans);
    }

    if (!dejaVuSans)
    {
        writeln("Could not create font: ", TTF_GetError());
        return 1;
    }
    /*
    auto white = SDL_Color(255, 255, 255);

    auto ledOnSurface = TTF_RenderUTF8_Solid(dejaVuSans, "Arduino says LED is ON.", white);
    auto ledOnTexture = SDL_CreateTextureFromSurface(renderer, ledOnSurface);
    auto ledOnRect = SDL_Rect(0, 0, ledOnSurface.w, ledOnSurface.h);
    scope (exit)
    {
        SDL_DestroyTexture(ledOnTexture);
        SDL_FreeSurface(ledOnSurface);
    }

    auto ledOffSurface = TTF_RenderUTF8_Solid(dejaVuSans, "Arduino says LED is OFF.", white);
    auto ledOffTexture = SDL_CreateTextureFromSurface(renderer, ledOffSurface);
    auto ledOffRect = SDL_Rect(0, 0, ledOffSurface.w, ledOffSurface.h);
    scope (exit)
    {
        SDL_DestroyTexture(ledOffTexture);
        SDL_FreeSurface(ledOffSurface);
    }*/

    spawn(&serialReceiveWorker, serialport);

    //SDL_ShowCursor(false);
    auto black = SDL_Color(0, 0, 0);
    auto red = SDL_Color(255, 0, 0);
    auto blue = SDL_Color(0, 0, 255);
    auto orange = SDL_Color(255, 165, 0);

    // List of the elements a part of the system (walls, fields, surfaces)
    SimulationElement[] elements = [
        //Honey
        new ViscousElement(292, 548, 144, 259, orange, 0.7),

        //Magnet Fields
        //Note: Strengths may need to be raised a ton or lowered a ton. I honestly dont know.
        new MagnetFieldElement(918, 607, 32, 30, red, 10,
                MagnetFieldElement.POLARITY.PUSH),
        new MagnetFieldElement(918, 679, 32, 30, blue, 10, MagnetFieldElement.POLARITY.PULL),

        //Outside Walls
        new ImpassableElement(289, 545, 249, 30, black),
        new ImpassableElement(535, 395, 30, 180, black),
        new ImpassableElement(535, 395, 323, 30, black),
        new ImpassableElement(855, 395, 30, 153, black),
        new ImpassableElement(855, 544, 224, 30, black),
        new ImpassableElement(1076, 544, 30, 293, black),
        new ImpassableElement(289, 807, 790, 30, black),
        new ImpassableElement(289, 545, 30, 265, black),

        //Inside Walls
        new ImpassableElement(678, 441, 30, 146, black),
        new ImpassableElement(572, 688, 232, 30, black),
        new ImpassableElement(918, 637, 32, 42, black),

        //Magnet-Walls
        new ImpassableElement(918, 607, 32, 30, red),
        new ImpassableElement(918, 679, 32, 30, blue)

    ];

    // Create the end effector representation in simulation
    EndEffector endEffector = new EndEffector();

    auto background = SDL_Rect(0, 0, 1300, 1000);

    // Create visual representation of the end effector (green square)
    // auto cursor = SDL_Rect(endEffector.x, endEffector.y, 10, 10);

    bool ledOn = false;

    // Event Loop
    while (true)
    {
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
                serialport.write([ledOn ? 1: 2]);
            }
        }

        receiveTimeout(-1.seconds, (immutable SerialMessage message) {
            ledOn = message.message[0] == 2;
        });



        // Clear render with a white background
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
        SDL_RenderFillRect(renderer, &background);
        SDL_RenderClear(renderer);



        // Draw each simulation element
        foreach (SimulationElement element; elements)
        {
            SDL_SetRenderDrawColor(renderer, element.colour.r,
                    element.colour.g, element.colour.b, SDL_ALPHA_OPAQUE);
            SDL_RenderFillRect(renderer, &element.rect);

            // If a collision is detected between endeffector and a sim. element,
            //  adjust the cursor location such that it doesn't disconnect with the visual element
            if (endEffector.detectCollision(element))
            {
                SDL_WarpMouseInWindow(window, endEffector.x, endEffector.y);
            }
        }

        // Update the cursor's x and y
        // cursor.x = endEffector.x;
        // cursor.y = endEffector.y;

        // Update the end effector's data (position, time)
        endEffector.update();

        // This setup is pretty janky, we definitely wanna change it to converging forces eventually
        auto currentPosition = new PositionVector(endEffector.x, endEffector.y);
        auto totalForce = new ForceVector(0, 0);
        auto currentForce = endEffector.calculateForce();
        //writeln(format("Force X: %f | ForceY: %f", currentForce.x, currentForce.y));

        // Draw the cursor
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, SDL_ALPHA_OPAQUE);
        endEffector.draw(renderer);

        // "Load" all elements to the render
        SDL_RenderPresent(renderer);

        // Check the magnetic forces
        foreach (SimulationElement element; elements)
        {
            if (cast(MagnetFieldElement) element)
            {
                auto force = element.force(currentPosition, currentForce);
                totalForce.x += force.x;
                totalForce.y += force.y;
                currentForce.x += force.x;
                currentForce.y += force.y;
            }
        }

        // Check the reactive forces from viscous elements
        foreach (SimulationElement element; elements)
        {
            if (cast(ViscousElement) element)
            {
                auto force = element.force(currentPosition, currentForce);
                totalForce.x += force.x;
                totalForce.y += force.y;
                currentForce.x += force.x;
                currentForce.y += force.y;
            }
        }

        // Check normal forces from impassable elements
        foreach (SimulationElement element; elements)
        {
            if (cast(ImpassableElement) element)
            {
                auto force = element.force(currentPosition, currentForce);
                totalForce.x += force.x;
                totalForce.y += force.y;
            }
        }
        if (!totalForce.isEmpty())
            stderr.writeln("Horizontal Force (N): " ~ to!string(
                    totalForce.x) ~ " | Vertical Force(N): " ~ to!string(totalForce.y));

        /*
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
        SDL_RenderClear(renderer);

        if (ledOn)
        {
            SDL_RenderCopy(renderer, ledOnTexture, null, &ledOnRect);
        }
        else
        {
            SDL_RenderCopy(renderer, ledOffTexture, null, &ledOffRect);
        }*/

    }
}
