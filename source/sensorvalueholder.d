import core.atomic : atomicLoad, atomicStore;

shared class SensorValueHolder
{
    private short leftEncoderValue;
    private short rightEncoderValue;
    private short leftCurrentSensorValue;
    private short rightCurrentSensorValue;

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

    @property short leftCurrentSensor()
    {
        return atomicLoad(this.leftCurrentSensorValue);
    }

    @property short leftCurrentSensor(short newval)
    {
        atomicStore(this.leftCurrentSensorValue, newval);
        return newval;
    }

    @property short rightCurrentSensor()
    {
        return atomicLoad(this.rightCurrentSensorValue);
    }

    @property short rightCurrentSensor(short newval)
    {
        atomicStore(this.rightCurrentSensorValue, newval);
        return newval;
    }
}