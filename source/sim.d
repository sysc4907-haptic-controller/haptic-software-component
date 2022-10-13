import bindbc.sdl;
import bindbcLoader = bindbc.loader.sharedlib;
import serial;

void hotkeyPressed(SDL_Keycode key)
{
    sendSerial("Hello World");
}
