import std.stdio;
import std.algorithm : max;

import core.sys.posix.sys.select;
import core.stdc.stdio : perror;

import serialport : SerialPortNonBlk;

// Wrapper Class for messages sent through network
immutable class NetworkType {
    string message;

    this(string message) {
        this.message = message;
    }

    public string toStringz() {
        return this.message;
    }
}