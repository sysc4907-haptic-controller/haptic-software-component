import bindbc.sdl;
import std.math;
import std.math.rounding : round;
import std.algorithm : max, min;
import std.datetime;
import std.stdio;
import std.format;
import core.time : MonoTime;

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
        return abs(x) < 0.001 && abs(y) < 0.001;
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

class VelocityVector : Vector
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

    // Draw function for the simulation element
    public void draw(SDL_Renderer* renderer)
    {
        SDL_SetRenderDrawColor(renderer, colour.r, colour.g, colour.b, SDL_ALPHA_OPAQUE);
        SDL_RenderFillRect(renderer, &rect);
    }
}

class GravelElement : SimulationElement
{
    this(int topLeftX, int topLeftY, uint length, uint height, SDL_Color colour)
    {
        super(topLeftX, topLeftY, length, height, colour);
    }
    override public ForceVector force(PositionVector endEffectorPos, ForceVector incEndEffectorForce)
    {
        return new ForceVector(0,0);
    }
}

class MovementElement : SimulationElement
{
    this(int topLeftX, int topLeftY, uint length, uint height, SDL_Color colour)
    {
        super(topLeftX, topLeftY, length, height, colour);
    }
    override public ForceVector force(PositionVector endEffectorPos, ForceVector incEndEffectorForce)
    {
        return new ForceVector(0,0);
    }
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
        auto midX = (rect.x + rect.x + rect.w) / 2;
        auto midY = (rect.y + rect.y + rect.h) / 2;
        auto dist = sqrt(pow(midX - endEffectorPos.x, 2) + pow(midY - endEffectorPos.y, 2));
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

    //ASSUMES FORCES ARE POSTIVE DOWN + RIGHT AND NEGATIVE UP + LEFT
    //ISSUE: If we take all force vectors and add them together, its possible this won't fully negate.
    override public ForceVector force(PositionVector endEffectorPos, ForceVector incEndEffectorForce)
    {
        switch (alongWall(endEffectorPos))
        {
        case EDGE.TOP:
            return new ForceVector(0, incEndEffectorForce.y > 0
                    ? -incEndEffectorForce.y : 0);
        case EDGE.BOTTOM:
            return new ForceVector(0, incEndEffectorForce.y < 0
                    ? -incEndEffectorForce.y : 0);
        case EDGE.LEFT:
            return new ForceVector(incEndEffectorForce.x > 0
                    ? -incEndEffectorForce.x : 0, 0);
        case EDGE.RIGHT:
            return new ForceVector(incEndEffectorForce.x < 0
                    ? -incEndEffectorForce.x : 0, 0);
        case EDGE.NONE:
        default:
            return new ForceVector(0, 0);
        }
    }
}

class EndEffector
{
    // The previous position (x,y) of the end effector
    public int prevX, prevY;

    // The current position (x,y) of the end effector
    public int x, y;

    public int RADIUS = 8;

    public VelocityVector prevVelocity, currVelocity;

    // The previous and current time
    MonoTime prevTime, currTime;

    this(int mouseX, int mouseY)
    {
        x = mouseX;
        y = mouseY;
        currTime = MonoTime.currTime();
        currVelocity = new VelocityVector(0, 0);
    }

    void update(PositionVector pos){
        update(cast(int) round(pos.x), cast(int) round(pos.y));
    }

    // Updates the position and time of the end effector accordingly
    void update(int newX, int newY)
    {
        prevX = x;
        prevY = y;
        prevTime = currTime;
        currTime = MonoTime.currTime();
        x = newX;
        y = newY;
        prevVelocity = currVelocity;
        double deltaTime = (currTime - prevTime).total!"msecs";
        currVelocity = new VelocityVector((x - prevX) * 0.1 * 1000 / deltaTime,
                (y - prevY) * 0.1 * 1000 / deltaTime);
    }

    // Calculate's the end effector's acceleration
    ForceVector calculateForce()
    {
        //note: acc is mm/ms*s -> m/s^2
        const MASS = 0.1; //0.1kg

        double deltaTime = (currTime - prevTime).total!"msecs";
        auto accX = (currVelocity.x - prevVelocity.x) / deltaTime;
        auto accY = (currVelocity.y - prevVelocity.y) / deltaTime;
        return new ForceVector(accX * MASS, accY * MASS);
    }

    bool collisionCalc(SimulationElement element, int x, int y)
    {
        if (cast(ImpassableElement) element)
        {
            PositionVector elemCenter = new PositionVector(element.rect.x + element.rect.w / 2.0,
                    element.rect.y + element.rect.h / 2.0);
            PositionVector difference = new PositionVector(x - elemCenter.x, y - elemCenter.y);
            PositionVector clamped = new PositionVector(max(-element.rect.w / 2.0,
                    min(element.rect.w / 2.0, difference.x)), max(-element.rect.h / 2.0,
                    min(element.rect.h / 2.0, difference.y)));
            difference = new PositionVector(elemCenter.x + clamped.x - x,
                    elemCenter.y + clamped.y - y);
            return sqrt(difference.x * difference.x + difference.y * difference.y) < RADIUS;
        }
        return false;
    }
    // Detects collision between end effector and an element
    bool detectCollision(SimulationElement element)
    {
        if (collisionCalc(element, x, y))
        {
            double tempX = x;
            double tempY = y;
            double m = (y - prevY) / (x - prevX + 0.00000000000001);
            double radians = atan(m);
            do
            {
                if (prevX > x)
                {
                    tempY -= sin(PI + radians);
                    tempX -= cos(PI + radians);
                }
                else
                {
                    tempY -= sin(radians);
                    tempX -= cos(radians);
                }
            }
            while (collisionCalc(element, cast(int) round(tempX), cast(int) round(tempY)));
            x = cast(int) round(tempX);
            y = cast(int) round(tempY);
            return true;
        }
        return false;
    }

    public void draw(SDL_Renderer* renderer)
    {
        /*
        for (int w = 0; w < RADIUS * 2; w++)
        {
            for (int h = 0; h < RADIUS * 2; h++)
            {
                int dx = RADIUS - w; // horizontal offset
                int dy = RADIUS - h; // vertical offset
                if ((dx*dx + dy*dy) <= (RADIUS * RADIUS))
                {
                    SDL_RenderDrawPoint(renderer, x + dx, y + dy);
                }
            }
        }
*/
        int updatingX = RADIUS - 1;
        int updatingY = 0;
        int tx = 1;
        int ty = 1;
        int error = tx - RADIUS * 2;

        while (updatingX >= updatingY)
        {
            //  Each of the following renders an octant of the circle
            SDL_RenderDrawPoint(renderer, x + updatingX, y - updatingY);
            SDL_RenderDrawPoint(renderer, x + updatingX, y + updatingY);
            SDL_RenderDrawPoint(renderer, x - updatingX, y - updatingY);
            SDL_RenderDrawPoint(renderer, x - updatingX, y + updatingY);
            SDL_RenderDrawPoint(renderer, x + updatingY, y - updatingX);
            SDL_RenderDrawPoint(renderer, x + updatingY, y + updatingX);
            SDL_RenderDrawPoint(renderer, x - updatingY, y - updatingX);
            SDL_RenderDrawPoint(renderer, x - updatingY, y + updatingX);

            if (error <= 0)
            {
                ++updatingY;
                error += ty;
                ty += 2;
            }
            if (error > 0)
            {
                --updatingX;
                tx += 2;
                error += (tx - RADIUS * 2);
            }
        }
    }
}
