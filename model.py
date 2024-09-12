from typing import Any
import numpy as np
from dataclasses import dataclass
from abc import  ABC, abstractmethod
from movement import Movement

VELOCITY = 1
OMEGA = np.pi/4
TURN_RIGHT:int = 1
GLOBAL_TURN_RIGHT:int = 1

@dataclass
class LocalFlags:
    l39:bool
    l41: bool
    initAngle: float = 0.0
    targetAngle: float = 0.0 

@dataclass
class Flags:
    updateGPS: bool = False
    autoDrive: bool = False
    checkPosition: bool = False
    updateCompass: bool = False
    continueMoving: bool = False
    localFlags:LocalFlags = LocalFlags(False, False)

    def print_state(self):
        print("****************************************")
        print(f"UpdateGPS = {self.updateGPS}\nautoDrive = {self.autoDrive}\ncheckPosition = {self.checkPosition}\nupdateCompass = {self.updateCompass}\ncontinueMoving = {self.continueMoving}")
        print("****************************************")
    
    @property
    def switch_updateGPS(self):
        if self.updateGPS:
            self.updateGPS = False
        else:
            self.updateGPS = True
    
    @property
    def switch_autoDrive(self):
        if self.autoDrive:
            self.autoDrive = False
        else:
            self.autoDrive = True
    
    @property
    def switch_updateCompass(self):
        if self.updateCompass:
            self.updateCompass = False
        else:
            self.updateCompass = True

    @property
    def switch_checkPosition(self):
        if self.checkPosition:
            self.checkPosition = False
        else:
            self.checkPosition = True

    @property
    def switch_continueMoving(self):
        if self.continueMoving:
            self.continueMoving = False
        else:
            self.continueMoving = True
    
    

@dataclass
class SetServo:
    angle: int = 105

    def set(self, angle):
        self.angle = angle


@dataclass
class MotorThread:
    flag: bool = False
    
    @property
    def switch(self):
        if self.flag:
            self.flag = False
        else:
            self.flag = True

@dataclass
class turnRight:
    flag: int = 1
    
    def set_flag(self, f):
        self.flag = -1


class State(ABC):
    @abstractmethod
    def __init__(self):
        raise NotImplementedError
    
    def __call__(self):
        raise NotImplementedError
    
    @abstractmethod
    def checkEntryCondition(self, flags):
        raise NotImplementedError 

    @abstractmethod
    def action(self):
        raise NotImplementedError



class State1(State):
    NAME = "Line 1 Condition"

    def __init__(self):
        self.entryFlags = Flags(
            checkPosition = True
        )
        
    def __call__(self, flags:Flags, motorThread:MotorThread, setServo:SetServo, rover:Movement):
        if self.checkEntryCondition(flags):
            print(f"Entered {self.NAME}")
            self.action(flags, motorThread, setServo, rover)
            return flags, motorThread, setServo, rover
        return flags, motorThread, setServo, rover

    def checkEntryCondition(self, flags:Flags):
        return  (not flags.autoDrive)\
                and (not flags.updateCompass)\
                and (flags.checkPosition)\
                and (not flags.continueMoving)
    
    def action(self, flags:Flags, motorThread: MotorThread, setServo:SetServo, rover:Movement):
        new_flags = flags
        new_flags.switch_updateCompass

        new_setServo = setServo

        new_motorThread = motorThread
        
        baseAngle = rover.getCompass()
        targetAngle = baseAngle + 0
        if new_flags.continueMoving or new_motorThread.flag:
            rover(VELOCITY, OMEGA, targetAngle)
        else: 
            rover(0,OMEGA, targetAngle)
        return new_flags, new_motorThread, new_setServo, rover
    
class State2(State):
    NAME = "Line 6 Condition"

    def __init__(self):
        self.entryFlags = Flags(
            continueMoving = True
        )
        
    def __call__(self, flags:Flags, motorThread:MotorThread, setServo:SetServo, rover:Movement):
        if self.checkEntryCondition(flags):
            print(f"Entered {self.NAME}")
            self.action(flags, motorThread, setServo, rover)
            return flags, motorThread, setServo, rover
        return flags, motorThread, setServo, rover

    def checkEntryCondition(self, flags:Flags):
        return  (not flags.autoDrive)\
                and (not flags.updateCompass)\
                and (not flags.checkPosition)\
                and (flags.continueMoving)
    
    def action(self, flags:Flags, motorThread: MotorThread, setServo:SetServo, rover:Movement):
        new_flags = flags
        new_flags.switch_continueMoving

        new_setServo = setServo
        new_setServo.set(105)

        new_motorThread = motorThread

        baseAngle = rover.getCompass()
        targetAngle = baseAngle
        if new_flags.continueMoving or new_motorThread.flag:
            rover(VELOCITY, OMEGA, targetAngle)
        else: 
            rover(0, OMEGA, targetAngle)

        return new_flags, new_motorThread, new_setServo

class State3(State):
    NAME = "Line 10 Condition"

    def __init__(self):
        self.entryFlags = Flags(
            continueMoving = True,
            checkPosition=True
        )
        
    def __call__(self, flags:Flags, motorThread:MotorThread, setServo:SetServo, rover:Movement):
        if self.checkEntryCondition(flags):
            print(f"Entered {self.NAME}")
            self.action(flags, motorThread, setServo, rover)
            return flags, motorThread, setServo, rover
        return flags, motorThread, setServo, rover

    def checkEntryCondition(self, flags:Flags):
        return  (not flags.autoDrive)\
                and (not flags.updateCompass)\
                and (flags.checkPosition)\
                and (flags.continueMoving)
    
    def action(self, flags:Flags, motorThread: MotorThread, setServo:SetServo, rover:Movement):
        new_flags = flags

        new_setServo = setServo

        new_motorThread = motorThread
        new_motorThread.flag = False

        baseAngle = rover.getCompass()
        targetAngle = baseAngle
        if new_flags.continueMoving or new_motorThread.flag:
            rover(VELOCITY, OMEGA, targetAngle)
        else: 
            rover(0, OMEGA, targetAngle)

        return new_flags, new_motorThread, new_setServo

class State4(State):
    NAME = "Line 13 Condition"

    def __init__(self):
        self.entryFlags = Flags()
        
    def __call__(self, flags:Flags, motorThread:MotorThread, setServo:SetServo, rover:Movement):
        if self.checkEntryCondition(flags):
            print(f"Entered {self.NAME}")
            self.action(flags, motorThread, setServo, rover)
            return flags, motorThread, setServo, rover
        return flags, motorThread, setServo, rover

    def checkEntryCondition(self, flags:Flags):
        return  (not flags.autoDrive)\
                and (not flags.updateCompass)\
                and (not flags.checkPosition)\
                and (not flags.continueMoving)
    
    def action(self, flags:Flags, motorThread: MotorThread, setServo:SetServo, rover:Movement):
        new_flags = flags

        new_setServo = setServo

        new_motorThread = motorThread
        new_motorThread.flag = False

        baseAngle = rover.getCompass()
        targetAngle = baseAngle
        if new_flags.continueMoving or new_motorThread.flag:
            rover(VELOCITY, OMEGA, targetAngle)
        else: 
            rover(0, OMEGA, targetAngle)

        return new_flags, new_motorThread, new_setServo


class State5(State):
    NAME = "Line 16 Condition"

    def __init__(self):
        self.entryFlags = Flags(
            updateCompass = True
        )
        
    def __call__(self, flags:Flags, motorThread:MotorThread, setServo:SetServo, rover:Movement):
        if self.checkEntryCondition(flags):
            print(f"Entered {self.NAME}")
            self.action(flags, motorThread, setServo, rover)
            return flags, motorThread, setServo, rover
        return flags, motorThread, setServo, rover

    def checkEntryCondition(self, flags:Flags):
        return  (not flags.autoDrive)\
                and (flags.updateCompass)
    
    def action(self, flags:Flags, motorThread: MotorThread, setServo:SetServo, rover:Movement):
        new_flags = flags
        new_flags.switch_updateCompass
        new_flags.switch_continueMoving

        new_setServo = setServo
        new_setServo.set(10)

        new_motorThread = motorThread

        baseAngle = rover.getCompass()
        # Provision to turn right
        targetAngle = baseAngle + TURN_RIGHT * np.pi/2
        if new_flags.continueMoving or new_motorThread.flag:
            rover(VELOCITY, OMEGA, targetAngle)
        else: 
            rover(0, OMEGA, targetAngle)

        return new_flags, new_motorThread, new_setServo


class State6(State):
    NAME = "Line 21 Condition"

    def __init__(self):
        self.entryFlags = Flags(
            autoDrive= True,
            updateGPS=True
        )
        self.track_distMoved = 0
        
        
    def __call__(self, flags:Flags, motorThread:MotorThread, setServo:SetServo, rover:Movement):
        if self.checkEntryCondition(flags):
            print(f"Entered {self.NAME}")
            self.action(flags, motorThread, setServo, rover)
            return flags, motorThread, setServo, rover
        return flags, motorThread, setServo, rover
    
    def checkEntryCondition(self, flags:Flags):
        return  (flags.autoDrive)\
                and (flags.checkPosition)
    
    def action(self, flags:Flags, motorThread: MotorThread, setServo:SetServo, rover:Movement):

        new_flags = flags
        new_setServo = setServo
        new_motorThread = motorThread

        start_coord = rover.getGPS()
        baseAngle = rover.getCompass()
        targetAngle = baseAngle
        if new_flags.continueMoving or new_motorThread.flag:
            rover(VELOCITY, OMEGA, targetAngle)
        else: 
            rover(0, OMEGA, targetAngle)
        end_coord = rover.getGPS()
        self.track_distMoved += ((start_coord[0] - end_coord[0])**2 + (start_coord[1] - end_coord[1])**2)**0.5
        print(self.track_distMoved)
        if self.track_distMoved <= 7:
            return new_flags, new_motorThread, new_setServo
        else:
            self.track_distMoved = 0
            new_flags.switch_checkPosition
            new_flags.switch_updateCompass
            return new_flags, new_motorThread, new_setServo
        


class State7(State):
    NAME = "Line 37 Condition"

    def __init__(self):
        self.entryFlags = Flags(
            autoDrive= True,
            updateGPS=True
        )
        self.track_angleMoved = 0
        self.localflags = LocalFlags(False, False)
        
    def __call__(self, flags:Flags, motorThread:MotorThread, setServo:SetServo, rover:Movement):
        if self.checkEntryCondition(flags):
            print(f"Entered {self.NAME}")
            self.action(flags, motorThread, setServo, rover)
            return flags, motorThread, setServo, rover
        return flags, motorThread, setServo, rover

    def checkEntryCondition(self, flags:Flags):
        return  (flags.autoDrive)\
                and (not flags.checkPosition)\
                and (flags.updateCompass)
    
    def action(self, flags:Flags, motorThread: MotorThread, setServo:SetServo, rover:Movement):
        global TURN_RIGHT
        new_flags = flags

        new_setServo = setServo

        new_motorThread = motorThread

        baseAngle = rover.getCompass()
        targetAngle = baseAngle + (TURN_RIGHT * np.deg2rad(70)) 
        if new_flags.continueMoving or new_motorThread.flag:
            rover(VELOCITY, OMEGA, targetAngle)
        else: 
            rover(0, OMEGA, targetAngle)

        if abs(rover.getCompass() - targetAngle) <= np.deg2rad(30):
            new_flags.localFlags.l39 = True
            new_flags.localFlags.initAngle = baseAngle
            new_flags.localFlags.targetAngle = targetAngle
            return new_flags, new_motorThread, new_setServo
        else:
            
            TURN_RIGHT = GLOBAL_TURN_RIGHT
            return new_flags, new_motorThread, new_setServo

class State7_B1(State):
    NAME = "Line 39 Condition"

    def __init__(self):
        self.entryFlags = Flags(
            autoDrive= True,
            updateGPS=True
        )
        self.track_angleMoved = 0
        self.target_angle = 0
        
        
    def __call__(self, flags:Flags, motorThread:MotorThread, setServo:SetServo, rover:Movement):
        if self.checkEntryCondition(flags):
            print(f"Entered {self.NAME}")
            self.target_angle = flags.localFlags.targetAngle
            self.action(flags, motorThread, setServo, rover)
            return flags, motorThread, setServo, rover
        return flags, motorThread, setServo, rover

    def checkEntryCondition(self, flags:Flags):
        return  (flags.autoDrive)\
                and (not flags.checkPosition)\
                and (flags.updateCompass)\
                and flags.localFlags.l39
    
    def action(self, flags:Flags, motorThread: MotorThread, setServo:SetServo, rover:Movement):
        global TURN_RIGHT
        new_flags = flags

        new_setServo = setServo

        new_motorThread = motorThread

        # baseAngle = rover.getCompass()
        # targetAngle = baseAngle
        if new_flags.continueMoving or new_motorThread.flag:
            rover(VELOCITY, OMEGA, self.target_angle)
        else: 
            rover(0, OMEGA, self.target_angle)

        if abs(rover.getCompass() - flags.localFlags.initAngle) >  abs(rover.getCompass() - flags.localFlags.targetAngle):
            baseAngle = rover.getCompass()
            targetAngle = baseAngle
            if new_flags.continueMoving or new_motorThread.flag:
                rover(VELOCITY, OMEGA, targetAngle)
            else: 
                rover(0, OMEGA, targetAngle)
            new_flags.switch_updateCompass
            new_flags.switch_updateGPS
            return new_flags, new_motorThread, new_setServo
        else:
            TURN_RIGHT = 1
            return new_flags, new_motorThread, new_setServo
        
class State8(State):
    NAME = "Line 57 Condition"

    def __init__(self):
        self.entryFlags = Flags(
            autoDrive= True,
            updateGPS=True
        )
        
    def __call__(self, flags:Flags, motorThread:MotorThread, setServo:SetServo, rover:Movement):
        if self.checkEntryCondition(flags):
            print(f"Entered {self.NAME}")
            self.action(flags, motorThread, setServo, rover)
            return flags, motorThread, setServo, rover
        return flags, motorThread, setServo, rover

    def checkEntryCondition(self, flags:Flags):
        return  (flags.autoDrive)\
                and (not flags.checkPosition)\
                and (not flags.updateCompass)\
                and (flags.updateGPS)
    
    def action(self, flags:Flags, motorThread: MotorThread, setServo:SetServo, rover:Movement):
        new_flags = flags
        new_flags.switch_updateGPS
        new_flags.switch_continueMoving

        new_setServo = setServo

        new_motorThread = motorThread

        baseAngle = rover.getCompass()
        # Provision to turn right
        targetAngle = baseAngle
        if new_flags.continueMoving or new_motorThread.flag:
            rover(VELOCITY, OMEGA, targetAngle)
        else: 
            rover(0, OMEGA, targetAngle)

        return new_flags, new_motorThread, new_setServo


class State9(State):
    NAME = "Line 66 Condition"

    def __init__(self):
        self.entryFlags = Flags(
            autoDrive= True,
            updateGPS=True
        )
        self.track_distMoved = 0
        
    def __call__(self, flags:Flags, motorThread:MotorThread, setServo:SetServo, rover:Movement):
        if self.checkEntryCondition(flags):
            print(f"Entered {self.NAME}")
            self.action(flags, motorThread, setServo, rover)
            return flags, motorThread, setServo, rover
        return flags, motorThread, setServo, rover

    def checkEntryCondition(self, flags:Flags):
        return  (flags.autoDrive)\
                and (not flags.checkPosition)\
                and (not flags.updateCompass)\
                and (not flags.updateGPS)\
                and (flags.continueMoving)
    
    def action(self, flags:Flags, motorThread: MotorThread, setServo:SetServo, rover:Movement):
        new_flags = flags
        new_setServo = setServo
        new_motorThread = motorThread

        start_coord = rover.getGPS()
        baseAngle = rover.getCompass()
        targetAngle = baseAngle
        if new_flags.continueMoving or new_motorThread.flag:
            rover(VELOCITY, OMEGA, targetAngle)
        else: 
            rover(0, OMEGA, targetAngle)
        end_coord = rover.getGPS()
        self.track_distMoved += ((start_coord[0] - end_coord[0])**2 + (start_coord[1] - end_coord[1])**2)**0.5
        print(self.track_distMoved)
        if self.track_distMoved <= 7:
            return new_flags, new_motorThread, new_setServo
        else:
            
            new_flags.switch_continueMoving
            new_flags.switch_autoDrive
            new_motorThread.flag = False
            self.track_distMoved = 0
            return new_flags, new_motorThread, new_setServo



class StateManager:
    def __init__(self, states):
        self.states = states

    def checkNextState(self, flags, motorThread, setServo, rover):
        
        for s in self.states:
            # print(f"check {s}")
            new_flags, new_motorThread, new_setServo, new_rover = s(flags, motorThread, setServo, rover)
            flags, motorThread, setServo, rover = new_flags, new_motorThread, new_setServo, new_rover
            # flags.print_state()
        return flags, motorThread, setServo, rover


rover = Movement()
flags = Flags(False, True, True, False, False)
motorThread = MotorThread()
motorThread.flag = True
setServo = SetServo()

s1 = State1()
s2 = State2()
s3 = State3()
s4 = State4()
s5 = State5()
s6 = State6()
s7 = State7()
s7_b1 = State7_B1()
s8 = State8()
s9 = State9()

man = StateManager([s1,s2,s3,s4,s5, s6, s7, s7_b1, s8, s9])


for i in range(50):

    flags, motorThread, setServo, rover = man.checkNextState(flags, motorThread, setServo, rover)

import matplotlib.pyplot as plt

# print(rover.history_x)
# print(rover.history_y)
plt.figure(figsize=(8, 8))
plt.plot(rover.history_x, rover.history_y, "-")
plt.title('Trajectory of the Point')
plt.xlabel('X position (m)')
plt.ylabel('Y position (m)')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.savefig("test.png")
# flags.print()
# flags_1 = Flags(True, True, True, True, True)
# flags_2 = Flags(True, True, True, True, True)

# print(flags == flags_1)
# print(flags == flags_2)