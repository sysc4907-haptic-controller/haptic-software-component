import std.stdio : write, writeln, readln;
import std.string : fromStringz;
import std.datetime : seconds;
import std.traits : select;
import std.concurrency : receiveTimeout, receive, spawn, Tid, thisTid, send;

import bindbc.sdl;
import bindbcLoader = bindbc.loader.sharedlib;
import serialport : SerialPortNonBlk;

import sim, serial, network;


void handleReceivedSerial(SerialType msg) {
    writeln("Message Received from Serial: " ~ msg.toStringz());
}

void handleReceivedNetwork(NetworkType msg) {
    writeln("Message Received from Network: " ~ msg.toStringz());
}

void eventLoop() {
    bool done = false;
    SDL_Event event;

    while(!done && SDL_PollEvent(&event)){
        switch(event.type){
           case SDL_KEYDOWN:
                hotkeyPressed(event.key.keysym.sym);
                break;
            case SDL_QUIT:
                done = true;
                break;
            default:
                break;
        }
        (-1.seconds).receiveTimeout(
            (SerialType msg) { handleReceivedSerial(msg); },
            (NetworkType msg) { handleReceivedNetwork(msg); }
        );
    }
}

void receiveLoop(string[] args) {
    setupSerial(args);
    while(true){
        //TODO: setup select to check for received serial, then output it ig?
        //IDK how select is useful here, cause if there is no incoming serial we wanna do nothing
        //Change to: if theres a serial msg incoming
        if(true){
            immutable SerialType receivedMessageFromSerial = getSerial(args);
            //send(mainTid, receivedMessageFromSerial);
        }
    }
}

static void startReceiveThread(Tid parentTid, string ip, string port){
    string[] args = new string[2];
    args[0] = ip;
    args[1] = port;
    receiveLoop(args);
}

int main(string[] args) {
    int error;
    error = SDL_Init(SDL_INIT_EVERYTHING);

    //TODO: Figure out which args are necessary and alter the spawn and the params to account for that
    spawn(&startReceiveThread, thisTid, args[0], args[1]);

    eventLoop();

    return 0;
}
