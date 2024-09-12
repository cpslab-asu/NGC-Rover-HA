from typing import Any, List
import numpy as np
from dataclasses import dataclass
from abc import  ABC, abstractmethod

from staliro.core.sample import Sample
from staliro.options import Options, SignalOptions
from staliro.specifications import RTAMTDense
from staliro.staliro import simulate_model
from staliro.signals import piecewise_constant
from staliro.core.interval import Interval
from staliro.core.model import Model, ModelInputs, Trace, ExtraResult
import numpy as np
from numpy.typing import NDArray

import numpy as np
import matplotlib.pyplot as plt


class Movement:
    def __init__(self):
        self.dt_integral = 0.01
        self.delta_t = 1
        self.history_t = [0.0]
        self.history_x = [0.0]
        self.history_y = [0.0]
        self.history_theta = [0.0]
        
    def __call__(self, v, omega):
        x = self.history_x[-1]
        y = self.history_y[-1]
        theta = self.history_theta[-1]

        for t in np.arange(0, 1, self.dt_integral):
            y += v * np.sin(theta) * self.dt_integral
            x += v * np.cos(theta) * self.dt_integral
            theta += omega * self.dt_integral
            
            # Store the positions
            self.history_x.append(x)
            self.history_y.append(y)
            self.history_theta.append(theta)
            self.history_t.append(self.history_t[-1]+self.dt_integral)
    
    def getCompass(self):
        return self.history_theta[-1]
    
    def getGPS(self):
        return (self.history_x[-1], self.history_y[-1])

@dataclass
class ManualActions:
    signal_66: bool = False
    signal_55: bool = False
    signal_turnRight = 0

@dataclass
class Flags:
    updateGPS: bool = False
    autoDrive: bool = False
    checkPosition: bool = False
    updateCompass: bool = False
    continueMoving: bool = False
    manualActions:ManualActions = ManualActions()

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
    
    def set_ManualActions(self, manualActions: ManualActions):
        self.manualActions = manualActions

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
    NAME = "State 1 Condition"

    def __init__(self):
        self.t = 0
        
    def __call__(self, flags:Flags, motorThread:MotorThread, setServo:SetServo, rover:Movement):
        if self.checkEntryCondition(flags):
            print(f"Entered {self.NAME}")
            self.action(flags, motorThread, setServo, rover)
            return flags, motorThread, setServo, rover
        return flags, motorThread, setServo, rover

    def checkEntryCondition(self, flags:Flags):
        return  (flags.checkPosition)\
                and (not flags.autoDrive)\
                and (not flags.updateCompass)\
                and (not flags.updateGPS)\
                and (not flags.continueMoving)\
    
    def action(self, flags:Flags, motorThread: MotorThread, setServo:SetServo, rover:Movement):
        new_flags = flags
        new_setServo = setServo
        new_motorThread = motorThread
        
        rover(0,0)
        if self.t < 5:
            self.t+=1
            return new_flags, new_motorThread, new_setServo, rover
        else:
            new_flags.switch_autoDrive
            self.t = 0
            return new_flags, new_motorThread, new_setServo, rover
        
class State2(State):
    NAME = "State 2 Condition"

    def __init__(self):
        self.dist_tracked = 0
        
    def __call__(self, flags:Flags, motorThread:MotorThread, setServo:SetServo, rover:Movement):
        if self.checkEntryCondition(flags):
            print(f"Entered {self.NAME}")
            self.action(flags, motorThread, setServo, rover)
            return flags, motorThread, setServo, rover
        return flags, motorThread, setServo, rover

    def checkEntryCondition(self, flags:Flags):
        return (flags.autoDrive)\
                and (flags.checkPosition)
    
    def action(self, flags:Flags, motorThread: MotorThread, setServo:SetServo, rover:Movement):
        new_flags = flags
        new_setServo = setServo
        new_motorThread = motorThread
        
        pre_movement = rover.getGPS()
        rover(VELOCITY, 0)
        post_movement = rover.getGPS()
        self.dist_tracked += ((pre_movement[0]-post_movement[0])**2 + (pre_movement[1]-post_movement[1])**2)**0.5

        if new_flags.manualActions.signal_66:
            new_flags.autoDrive = False
            new_flags.checkPosition = False
            return new_flags, new_motorThread, new_setServo, rover
        
        if self.dist_tracked < 7:
            return new_flags, new_motorThread, new_setServo, rover
        else:
            new_flags.switch_checkPosition
            new_flags.switch_updateCompass
            self.dist_tracked = 0
            return new_flags, new_motorThread, new_setServo, rover
        

class State3(State):
    NAME = "State 3 Condition"

    def __init__(self):
        self.angle_moved = 0
        
    def __call__(self, flags:Flags, motorThread:MotorThread, setServo:SetServo, rover:Movement):
        if self.checkEntryCondition(flags):
            print(f"Entered {self.NAME}")
            self.action(flags, motorThread, setServo, rover)
            return flags, motorThread, setServo, rover
        return flags, motorThread, setServo, rover

    def checkEntryCondition(self, flags:Flags):
        return (flags.autoDrive)\
                and (not flags.checkPosition)\
                and (flags.updateCompass)
    
    def action(self, flags:Flags, motorThread: MotorThread, setServo:SetServo, rover:Movement):
        new_flags = flags
        new_setServo = setServo
        new_motorThread = motorThread
        
        val_pre_movement = rover.getCompass()
        rover(VELOCITY, new_flags.manualActions.signal_turnRight*OMEGA)
        self.angle_moved += abs(val_pre_movement-rover.getCompass())
        if new_flags.manualActions.signal_66:
            new_flags.autoDrive = False
            new_flags.checkPosition = False
            return new_flags, new_motorThread, new_setServo, rover
        
        if self.angle_moved < np.deg2rad(70):
            return new_flags, new_motorThread, new_setServo, rover
        else:
            new_flags.switch_updateCompass
            new_flags.switch_updateGPS
            
            self.angle_moved = 0
            return new_flags, new_motorThread, new_setServo, rover


class State4(State):
    NAME = "State 4 Condition"

    def __init__(self):
        pass
        
    def __call__(self, flags:Flags, motorThread:MotorThread, setServo:SetServo, rover:Movement):
        if self.checkEntryCondition(flags):
            print(f"Entered {self.NAME}")
            self.action(flags, motorThread, setServo, rover)
            return flags, motorThread, setServo, rover
        return flags, motorThread, setServo, rover

    def checkEntryCondition(self, flags:Flags):
        return (flags.autoDrive)\
                and (not flags.checkPosition)\
                and (not flags.updateCompass)\
                and (flags.updateGPS)
    
    def action(self, flags:Flags, motorThread: MotorThread, setServo:SetServo, rover:Movement):
        new_flags = flags
        new_setServo = setServo
        new_motorThread = motorThread
        
        
        rover(VELOCITY, new_flags.manualActions.signal_turnRight*OMEGA)

        new_flags.switch_updateGPS
        new_flags.continueMoving = True
        return new_flags, new_motorThread, new_setServo, rover

class State5(State):
    NAME = "State 5 Condition"

    def __init__(self):
        self.dist_tracked = 0
        
    def __call__(self, flags:Flags, motorThread:MotorThread, setServo:SetServo, rover:Movement):
        if self.checkEntryCondition(flags):
            print(f"Entered {self.NAME}")
            self.action(flags, motorThread, setServo, rover)
            return flags, motorThread, setServo, rover
        return flags, motorThread, setServo, rover

    def checkEntryCondition(self, flags:Flags):
        return (flags.autoDrive)\
                and (not flags.checkPosition)\
                and (not flags.updateCompass)\
                and (not flags.updateGPS)\
                and (flags.continueMoving)
    
    def action(self, flags:Flags, motorThread: MotorThread, setServo:SetServo, rover:Movement):
        new_flags = flags
        new_setServo = setServo
        new_motorThread = motorThread
        
        
        pre_movement = rover.getGPS()
        rover(VELOCITY, 0)
        post_movement = rover.getGPS()
        self.dist_tracked += ((pre_movement[0]-post_movement[0])**2 + (pre_movement[1]-post_movement[1])**2)**0.5
        if new_flags.manualActions.signal_66:
            new_flags.autoDrive = False
            new_flags.checkPosition = False
            return new_flags, new_motorThread, new_setServo, rover
        
        if self.dist_tracked < 7:
            return new_flags, new_motorThread, new_setServo, rover
        else:
            new_flags.switch_autoDrive
            new_flags.switch_continueMoving
            self.dist_tracked = 0
            return new_flags, new_motorThread, new_setServo, rover
        

class State6(State):
    NAME = "State 6 Condition"

    def __init__(self):
        pass
        
    def __call__(self, flags:Flags, motorThread:MotorThread, setServo:SetServo, rover:Movement):
        if self.checkEntryCondition(flags):
            print(f"Entered {self.NAME}")
            self.action(flags, motorThread, setServo, rover)
            return flags, motorThread, setServo, rover
        return flags, motorThread, setServo, rover

    def checkEntryCondition(self, flags:Flags):
        return (not flags.autoDrive)\
                and (not flags.checkPosition)\
                and (not flags.updateCompass)\
                and (not flags.continueMoving)
    
    def action(self, flags:Flags, motorThread: MotorThread, setServo:SetServo, rover:Movement):
        new_flags = flags
        new_setServo = setServo
        new_motorThread = motorThread
        
        
        rover(0, 0)
        return new_flags, new_motorThread, new_setServo, rover    


class State7(State):
    NAME = "State 7 Condition"

    def __init__(self):
        pass
        
    def __call__(self, flags:Flags, motorThread:MotorThread, setServo:SetServo, rover:Movement):
        if self.checkEntryCondition(flags):
            print(f"Entered {self.NAME}")
            self.action(flags, motorThread, setServo, rover)
            return flags, motorThread, setServo, rover
        return flags, motorThread, setServo, rover

    def checkEntryCondition(self, flags:Flags):
        return (not flags.autoDrive)\
                and (not flags.checkPosition)\
                and (not flags.updateCompass)\
                and (flags.continueMoving)
    
    def action(self, flags:Flags, motorThread: MotorThread, setServo:SetServo, rover:Movement):
        new_flags = flags
        new_setServo = setServo
        new_motorThread = motorThread
        
        
        rover(VELOCITY, 0)
        if new_flags.manualActions.signal_55:
            new_flags.checkPosition = True
            return new_flags, new_motorThread, new_setServo, rover
        
        new_flags.switch_continueMoving
        return new_flags, new_motorThread, new_setServo, rover  


class State8(State):
    NAME = "State 8 Condition"

    def __init__(self):
        pass
        
    def __call__(self, flags:Flags, motorThread:MotorThread, setServo:SetServo, rover:Movement):
        if self.checkEntryCondition(flags):
            print(f"Entered {self.NAME}")
            self.action(flags, motorThread, setServo, rover)
            return flags, motorThread, setServo, rover
        return flags, motorThread, setServo, rover

    def checkEntryCondition(self, flags:Flags):
        return (not flags.autoDrive)\
                and (flags.updateCompass)
    
    def action(self, flags:Flags, motorThread: MotorThread, setServo:SetServo, rover:Movement):
        new_flags = flags
        new_setServo = setServo
        new_motorThread = motorThread
        
        
        rover(VELOCITY, OMEGA)
        
        new_flags.switch_updateCompass
        new_flags.switch_continueMoving
        return new_flags, new_motorThread, new_setServo, rover  


class State9(State):
    NAME = "State 9 Condition"

    def __init__(self):
        pass
        
    def __call__(self, flags:Flags, motorThread:MotorThread, setServo:SetServo, rover:Movement):
        if self.checkEntryCondition(flags):
            print(f"Entered {self.NAME}")
            self.action(flags, motorThread, setServo, rover)
            return flags, motorThread, setServo, rover
        return flags, motorThread, setServo, rover

    def checkEntryCondition(self, flags:Flags):
        return (not flags.autoDrive)\
                and (not flags.updateCompass)\
                and (flags.checkPosition)\
                and (flags.continueMoving)
    
    def action(self, flags:Flags, motorThread: MotorThread, setServo:SetServo, rover:Movement):
        new_flags = flags
        new_setServo = setServo
        new_motorThread = motorThread
        
        rover(0, 0)
        return new_flags, new_motorThread, new_setServo, rover  
    

class StateManager:
    def __init__(self):
        

        state_1 = State1()
        state_2 = State2()
        state_3 = State3()
        state_4 = State4()
        state_5 = State5()
        state_6 = State6()
        state_7 = State7()
        state_8 = State8()
        state_9 = State9()
        self.states = [state_1, state_2, state_3, state_4, state_5, state_6, state_7, state_8, state_9]
        
        
    def __call__(self, manualActions:List[ManualActions], time:int = 50):

        self.rover = Movement()
        self.flags = Flags(False, False, True, False, False)
        self.flags.set_ManualActions(manualActions[0])

        self.motorThread = MotorThread()
        self.motorThread.flag = True
        self.setServo = SetServo()

        for i in range(time):
            self.flags, self.motorThread, self.setServo, self.rover = self.checkNextState(self.flags, self.motorThread, self.setServo, self.rover)
        return self.flags, self.motorThread, self.setServo, self.rover
    
    def checkNextState(self, flags, motorThread, setServo, rover):
        

        for s in self.states:
            new_flags, new_motorThread, new_setServo, new_rover = s(flags, motorThread, setServo, rover)
            flags, motorThread, setServo, rover = new_flags, new_motorThread, new_setServo, new_rover
        return flags, motorThread, setServo, rover


def generateManualActions(signal_55, signal_66, signal_turnRight):
    manualActionList = []
    for s_55, s_66, s_tr in zip(signal_55, signal_66, signal_turnRight):
        ma = ManualActions()
        if s_55:
            ma.signal_55 = True
        if s_66:
            ma.signal_66 = True
        if s_tr == 1:
            ma.signal_turnRight = 1
        elif s_tr == -1:
            ma.signal_turnRight = -1
        manualActionList.append(ma)
    return manualActionList
    


RoverHADataT = NDArray[np.float_]
RoverHAResultT = ExtraResult[RoverHADataT, RoverHADataT]


class RoverHAModel(Model[RoverHAResultT, None]):

    def __init__(self) -> None:
        self.sampling_step = 1

    def simulate(self, inputs:ModelInputs, intrvl: Interval) -> RoverHAResultT:
        n_times = (intrvl.length // self.sampling_step)
        signal_times = np.linspace(intrvl.lower, intrvl.upper, int(n_times))
        inputs_bounds = [i*intrvl.length for i in inputs.static]
        signal_55 = np.zeros_like(signal_times)
        for i, t in enumerate(np.sort(inputs_bounds[:4])):
            if i % 2 == 0:
                signal_55[signal_times >= t] = 1  # Set signal to 1 at even indices (rise)
            else:
                signal_55[signal_times >= t] = 0  # Set signal to 0 at odd indices (fall)

        signal_66 = np.zeros_like(signal_times)
        for i, t in enumerate(np.sort(inputs_bounds[4:8])):
            if i % 2 == 0:
                signal_66[signal_times >= t] = 1  # Set signal to 1 at even indices (rise)
            else:
                signal_66[signal_times >= t] = 0  # Set signal to 0 at odd indices (fall)

        signal_tr = np.zeros_like(signal_times)-1
        for i, t in enumerate(np.sort(inputs_bounds[8:])):
            if i % 2 == 0:
                signal_tr[signal_times >= t] = 1  # Set signal to 1 at even indices (rise)
            else:
                signal_tr[signal_times >= t] = -1  # Set signal to 0 at odd indices (fall)


        man = StateManager()
        manualActions:List[ManualActions] = generateManualActions(signal_55=signal_55, signal_66=signal_66, signal_turnRight=signal_tr)
        flags, motorThread, setServo, rover = man(manualActions, T)

        
        data_array = np.array([rover.history_x, rover.history_y, rover.history_theta])
        timestamps_list = np.array(rover.history_t).flatten()
        
        trace = Trace(timestamps_list, data_array.T)

        inTrace = Trace(signal_times, [signal_55, signal_66])
        return RoverHAResultT(trace, inTrace)
    


if __name__ == "__main__":
    
    VELOCITY = 1
    OMEGA = np.pi/4
    
    rng = np.random.default_rng(12346)
    T = 50 


    init_conditions = [
                        [0,1],[0,1],[0,1],[0,1], 
                        [0,1],[0,1],[0,1],[0,1],
                        [0,1],[0,1],[0,1],[0,1]
                    ]
    options = Options(runs=1, iterations=1, interval=(0, T), signals = [], static_parameters=init_conditions)


    # signal_bounds = sum((signal.control_points for signal in signals), ())
    bounds = options.static_parameters
    rng = np.random.default_rng(12345)
    sample = Sample([rng.uniform(bound.lower, bound.upper) for bound in bounds])

    model = RoverHAModel()
    res = simulate_model(model, options, sample)

    data = np.array(res.trace.states)
    plt.figure(figsize=(8, 8))
    plt.plot(data[:,0], data[:,1], "-")
    plt.title('Trajectory of the Point')
    plt.xlabel('X position (m)')
    plt.ylabel('Y position (m)')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.savefig("test_ozmen.png")
