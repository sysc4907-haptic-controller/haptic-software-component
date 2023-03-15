import core.atomic : atomicLoad, atomicStore;

shared class SensorValueHolder
{
    private short leftEncoderValue;
    private short rightEncoderValue;
    //private ubyte leftCurrentSensorValue;
    //private ubyte rightCurrentSensorValue;

    @property short leftEncoder()
    {
        return atomicLoad(this.leftEncoderValue);
    }

    @property short leftEncoder(short newval)
    {
        atomicStore(this.leftEncoderValue, newval);
        return newval;
    }

    @property short rightEncoder()
    {
        return atomicLoad(this.rightEncoderValue);
    }

    @property short rightEncoder(short newval)
    {
        atomicStore(this.rightEncoderValue, newval);
        return newval;
    }
}