#!/usr/bin/env python

import math

maxSpeed       = 2.0
maxTurnSpeed   = 1.0
robotWidth     = 0.6
safetyMargin   = 0.4
extraMargin    = 0.1
turnIntensity  = 1.7
turnResistance = 1.5
minImpactTime  = 2.0

def sign(x):
    return -1.0 if x < 0.0 else 1.0

def d(width, alpha, theta):
    if (theta <= alpha):
        div = (2.0 * math.sin(alpha - theta))
        if (div == 0.0):
            return float('+inf')
        else:
            return (width * math.sin(math.pi / 2.0 - alpha)) / div
    else:
        return d(width, -alpha, -theta)

def minDist(width, alphaLeft, alphaRight, ranges):
    def corridorDist((theta, scan)):
        if (theta < alphaRight):
            if (scan < d(width, alphaRight, theta)):
                return scan
            else:
                return float('+inf')
        elif (theta > alphaRight and theta < alphaLeft):
            return scan
        else: #(theta > alphaLeft)
            if (scan < d(width, alphaLeft, theta)):
                return scan
            else:
                return float('+inf')

    return min(map(corridorDist, ranges))

def forwardSpeed(width, alpha, goalDist, goalFlag, ranges):
    global safetyMargin, minImpactTime
    
    alphaLeft = max(0.0, alpha)
    alphaRight = min(0.0, alpha)
    safeDist = minDist(width, alphaLeft, alphaRight, ranges) - \
               safetyMargin * 2.0
    distance = min(goalDist, safeDist) if goalFlag else safeDist

    return min(maxSpeed, distance / minImpactTime)

def turnSpeed(alpha):
    global maxTurnSpeed, turnIntensity
    
    ret = sign(alpha) * maxTurnSpeed
    return ret * (2.0 * abs(alpha / math.pi) ** (1.0 / turnIntensity))

def goalProgress(width, goalAlpha, goalDist, alpha, ranges):
    global extraMargin, turnResistance

    x = min(goalDist, minDist(width + extraMargin, alpha, alpha, ranges))
    return x * (math.cos(abs(goalAlpha - alpha)) ** (turnIntensity + 0.0j)).real

def findDirection(width, goalAlpha, goalDist, ranges):
    def findBetterAlpha((progAlpha, prog), alpha):
        newProg = goalProgress(width, goalAlpha, goalDist, alpha, ranges)
        if (newProg > prog):
            return (alpha, newProg)
        else:
            return (progAlpha, prog)

    angles = [p[0] for p in ranges]
    return reduce(findBetterAlpha, angles, (0.0, float("-inf")))[0]

def woahForward(goalAlpha, goalDist, goalFlag, ranges):
    global robotWidth, safetyMargin

    width = robotWidth + safetyMargin
    alpha = findDirection(width, goalAlpha, goalDist, ranges)
    speed = forwardSpeed(width, alpha, goalDist, goalFlag, ranges)
    turn = turnSpeed(alpha)

    return (speed, turn)

def woahBackward(goalAlpha):
    global maxTurnSpeed
    
    return (0.0, sign(goalAlpha) * maxTurnSpeed)

def woah(goalAlpha, goalDist, goalFlag, ranges, senseBounds):
    if (goalAlpha >= senseBounds[0] and goalAlpha <= senseBounds[1]):
        return woahForward(goalAlpha, goalDist, goalFlag, ranges)
    else:
        return woahBackward(goalAlpha)

def woah_ahead(alpha, speed, msg):
    global maxSpeed, maxTurnSpeed

    maxTurnSpeed = alpha
    maxSpeed     = speed

    ranges = []
    angMin = msg.angle_max
    angMax = msg.angle_min
    startAngle = msg.angle_min

    for i in xrange(len(msg.ranges)):
        pair = (ang, r) = (msg.angle_min + float(i) * 
                           msg.angle_increment, msg.ranges[i])

        if (r < msg.range_min or r > msg.range_max):
            pass
        else:
            angMin = min(angMin, ang)
            angMax = max(angMax, ang)
            ranges.append(pair)

    return woah(alpha, speed * 2.0, True, ranges, [angMin, angMax])

