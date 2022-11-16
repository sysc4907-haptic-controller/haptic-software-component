import std.stdio : write, writeln, readln, stderr;
import std.string : fromStringz;
import std.datetime : seconds;
import std.variant : Variant;
import std.concurrency : receiveTimeout, receive, spawn, Tid, thisTid, send;
import std.file;
import std.conv : to;

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
    auto sdlTTFLoadResult = loadSDLTTF();

    auto error = SDL_Init(SDL_INIT_VIDEO);

    const auto windowWidth = 1300;
    const auto windowHeight = 1000;

    auto window = SDL_CreateWindow("2-DOF Hybrid Haptic Device",
            SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, windowWidth, windowHeight, 0);
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");

    auto renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    scope (exit)
    {
        SDL_DestroyRenderer(renderer);
    }
    shared serialport = new shared SerialPort(args[1], 115200);
    // TODO: See if we can get this to work
    // scope (exit)
    // {
    //     serialport.close();
    // }

    error = TTF_Init();
    auto dejaVuSans = TTF_OpenFont("DejaVuSans.ttf", 48);
    scope (exit)
    {
        TTF_CloseFont(dejaVuSans);
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

    auto black = SDL_Color(0, 0, 0);
    auto red = SDL_Color(255, 0, 0);
    auto blue = SDL_Color(0, 0, 255);
    auto orange = SDL_Color(255, 165, 0);

    SimulationElement[] elements = [
        //Outside Walls
        new ImpassableElement(289, 545, 249, 3, black),
        new ImpassableElement(535, 395, 3, 153, black),
        new ImpassableElement(535, 395, 323, 3, black),
        new ImpassableElement(855, 395, 3, 153, black),
        new ImpassableElement(855, 544, 224, 4, black),
        new ImpassableElement(1076, 544, 3, 267, black),
        new ImpassableElement(289, 807, 790, 3, black),
        new ImpassableElement(289, 545, 3, 265, black),

        //Inside Walls
        new ImpassableElement(678, 441, 4, 146, black),
        new ImpassableElement(572, 688, 232, 4, black),
        new ImpassableElement(918, 637, 32, 42, black),

        //Magnet-Walls
        new ImpassableElement(918, 607, 32, 30, red),
        new ImpassableElement(918, 679, 32, 30, blue),

        //Honey
        new ViscousElement(292, 548, 144, 259, orange, 0.7),

        //Magnet Fields
        //Note: Strengths may need to be raised a ton or lowered a ton. I honestly dont know.
        new MagnetFieldElement(918, 607, 32, 30, red, 10,
                MagnetFieldElement.POLARITY.PUSH),
        new MagnetFieldElement(918, 679, 32, 30, blue, 10, MagnetFieldElement.POLARITY.PULL)
    ];

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
    auto background = SDL_Rect(0, 0, 1300, 1000);
    SDL_RenderFillRect(renderer, &background);
    SDL_RenderClear(renderer);
    foreach (SimulationElement element; elements)
    {
        SDL_SetRenderDrawColor(renderer, element.colour.r, element.colour.g,
                element.colour.b, SDL_ALPHA_OPAQUE);
        SDL_RenderFillRect(renderer, &element.rect);
    }
    SDL_RenderPresent(renderer);
    bool ledOn = false;
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

        //TODO: @will instead of these use the mouse pos/arrows or whatever u choose to use
        // This setup is pretty janky, we definitely wanna change it to converging forces eventually
        auto currentPosition = new PositionVector(0, 0);
        auto currentForce = new ForceVector(0, 0);

        ForceVector totalForce = new ForceVector(0, 0);
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
            stderr.writeln("Horizontal Force: " ~ to!string(
                    totalForce.x) ~ " | Vertical Force: " ~ to!string(totalForce.y));

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
