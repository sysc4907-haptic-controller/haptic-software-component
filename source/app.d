import std.stdio : write, writeln, readln, stderr;
import std.string : fromStringz;
import std.datetime : seconds;
import std.variant : Variant;
import std.concurrency : receiveTimeout, receive, spawn, Tid, thisTid, send;
import std.file;

import bindbc.sdl;
import bindbcLoader = bindbc.loader.sharedlib;
import serialport : SerialPort;

import serial : serialReceiveWorker, SerialMessage;

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

    const auto windowWidth = 1280;
    const auto windowHeight = 780;

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
    }

    spawn(&serialReceiveWorker, serialport);

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

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
        SDL_RenderClear(renderer);

        if (ledOn)
        {
            SDL_RenderCopy(renderer, ledOnTexture, null, &ledOnRect);
        }
        else
        {
            SDL_RenderCopy(renderer, ledOffTexture, null, &ledOffRect);
        }

        SDL_RenderPresent(renderer);
    }
}
