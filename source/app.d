import std.stdio : write, writeln, readln;
import std.string : fromStringz;
import std.datetime : seconds;
import std.traits : select;
import std.concurrency : receiveTimeout, receive, spawn, Tid, thisTid, send;

import bindbc.sdl;
import bindbcLoader = bindbc.loader.sharedlib;
import serialport : SerialPortNonBlk;

import serial, network;

void handleReceivedSerial(SerialType msg)
{
    writeln("Message Received from Serial: " ~ msg.toStringz());
}

void handleReceivedNetwork(NetworkType msg)
{
    writeln("Message Received from Network: " ~ msg.toStringz());
}

void hotkeyPressed(SDL_Keycode key)
{
    sendSerial("Hello World");
}

void eventLoop()
{
    bool done = false;
    SDL_Event event;

    while (!done && SDL_PollEvent(&event))
    {
        switch (event.type)
        {
        case SDL_KEYDOWN:
            hotkeyPressed(event.key.keysym.sym);
            break;
        case SDL_QUIT:
            done = true;
            break;
        default:
            break;
        }
        (-1.seconds).receiveTimeout((SerialType msg) {
            handleReceivedSerial(msg);
        }, (NetworkType msg) { handleReceivedNetwork(msg); });
    }
}

void receiveSerialLoop(Tid parentTid, string serialPort)
{
    while (true)
    {
        //Change to: if theres a serial msg incoming
        if (true)
        {
            //Change to: wait for a serial receive
            //TODO: @will move code from serial to this function
            immutable SerialType receivedMessageFromSerial = null;
            send(parentTid, receivedMessageFromSerial);
        }
    }
}

void receiveNetworkLoop(Tid parentTid, string networkPort)
{
    while (true)
    {
        //Change to: if theres a network msg incoming
        if (true)
        {
            //Change to: wait for a network receive
            immutable NetworkType receivedMessageFromNetwork = null;
            send(parentTid, receivedMessageFromNetwork);
        }
    }
}

//args: serialPort, networkPort, otherIP
int main(string[] args)
{
    int error;
    error = SDL_Init(SDL_INIT_EVERYTHING);

    //TODO: Figure out which args are necessary and alter the spawn and the params to account for that
    spawn(&receiveSerialLoop, thisTid, args[0]);
    spawn(&receiveNetworkLoop, thisTid, args[1]);
    eventLoop();

    return 0;
}
