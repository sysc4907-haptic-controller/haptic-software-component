import bindbc.sdl;
import std.math;

class Vector
{
    public double x;
    public double y;
    this(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    public bool isEmpty()
    {
        return x < 0.001 && y < 0.001;
    }
}

class ForceVector : Vector
{
    this(double x, double y)
    {
        super(x, y);
    }
}

class PositionVector : Vector
{
    this(double x, double y)
    {
        super(x, y);
    }
}

class SimulationElement
{
    //Fields
    public const SDL_Rect rect;
    public const SDL_Color colour;

    //Methods
    this(int topLeftX, int topLeftY, uint length, uint height, SDL_Color colour)
    {
        this.rect = SDL_Rect(topLeftX, topLeftY, length, height);
        this.colour = colour;
    }

    abstract public ForceVector force(PositionVector endEffectorPos, ForceVector incEndEffectorForce);
}

class MagnetFieldElement : SimulationElement
{
    //SO FOR NOW WHEN A MAGNET IS CREATED A CORRESPONDING WALL NEEDS TO BE CREATED WHERE THE MAGNET WOULD EXIST
    //The SimElem fields are used to find calculate the distance from the effector atm
    //In the future we need to integrate the two of them, once we can do converging forces
    public enum POLARITY
    {
        PUSH,
        PULL
    }

    public double strength;
    public POLARITY polarity;
    this(int topLeftX, int topLeftY, uint length, uint height, SDL_Color colour,
            double strength, POLARITY polarity)
    {
        super(topLeftX, topLeftY, length, height, colour);
        this.strength = strength;
        this.polarity = polarity;
    }

    override public ForceVector force(PositionVector endEffectorPos, ForceVector incEndEffectorForce)
    {
        auto midX = (rect.x + rect.w) / 2;
        auto midY = (rect.y + rect.h) / 2;
        auto dist = sqrt(pow(abs(midX - endEffectorPos.x), 2) + pow(abs(midY - endEffectorPos.y), 2));
        auto angle = atan2(midY - endEffectorPos.y, midX - endEffectorPos.x);
        if (polarity == POLARITY.PULL)
        {
            return new ForceVector(strength * (1 / pow(dist, 2)) * cos(angle),
                    strength * (1 / pow(dist, 2)) * sin(angle));
        }
        else
        {
            return new ForceVector(-strength * (1 / pow(dist, 2)) * cos(angle),
                    -strength * (1 / pow(dist, 2)) * sin(angle));
        }

    }
}

class ViscousElement : SimulationElement
{
    //0-1
    public double coefficient;

    this(int topLeftX, int topLeftY, uint length, uint height, SDL_Color colour, double coefficient)
    {
        super(topLeftX, topLeftY, length, height, colour);
        this.coefficient = coefficient;
    }

    override public ForceVector force(PositionVector endEffectorPos, ForceVector incEndEffectorForce)
    {
        if (endEffectorPos.y >= rect.y && endEffectorPos.y <= rect.y + rect.h
                && endEffectorPos.x >= rect.x && endEffectorPos.x <= rect.x + rect.w)
        {
            return new ForceVector(incEndEffectorForce.x * coefficient - incEndEffectorForce.x,
                    incEndEffectorForce.y * coefficient - incEndEffectorForce.y);
        }
        return new ForceVector(0, 0);
    }
}

class ImpassableElement : SimulationElement
{
    enum EDGE
    {
        TOP,
        BOTTOM,
        LEFT,
        RIGHT,
        NONE
    }

    this(int topLeftX, int topLeftY, uint length, uint height, SDL_Color colour)
    {
        super(topLeftX, topLeftY, length, height, colour);
    }

    private EDGE alongWall(PositionVector pos)
    {
        if (pos.y >= rect.y && pos.y <= rect.y + rect.h && pos.x >= rect.x && pos.x
                <= rect.x + rect.w)
        {
            return closestEdge(pos);
        }
        return EDGE.NONE;
    }

    private EDGE closestEdge(PositionVector pos)
    {
        auto distances = [
            abs(rect.x - pos.x), abs(rect.x + rect.w - pos.x),
            abs(rect.y - pos.y), abs(rect.y + rect.h - pos.y)
        ];
        double min = abs(rect.x - pos.x);
        ulong edge = 0;
        foreach (i, double dist; distances)
        {
            if (dist < min)
            {
                min = dist;
                edge = i;
            }
        }
        return [EDGE.LEFT, EDGE.RIGHT, EDGE.TOP, EDGE.BOTTOM][edge];
    }

    //ASSUMES FORCES ARE POSTIVE UP + RIGHT AND NEGATIVE DOWN + LEFT
    //ISSUE: If we take all force vectors and add them together, its possible this won't fully negate.
    override public ForceVector force(PositionVector endEffectorPos, ForceVector incEndEffectorForce)
    {
        switch (alongWall(endEffectorPos))
        {
        case EDGE.TOP:
            return new ForceVector(0, incEndEffectorForce.y < 0
                    ? -incEndEffectorForce.y : 0);
        case EDGE.BOTTOM:
            return new ForceVector(0, incEndEffectorForce.y > 0
                    ? -incEndEffectorForce.y : 0);
        case EDGE.LEFT:
            return new ForceVector(incEndEffectorForce.x > 0
                    ? -incEndEffectorForce.y : 0, 0);
        case EDGE.RIGHT:
            return new ForceVector(incEndEffectorForce.x < 0
                    ? -incEndEffectorForce.y : 0, 0);
        case EDGE.NONE:
        default:
            return new ForceVector(0, 0);
        }
    }
}
