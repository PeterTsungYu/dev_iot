#import matplotlib.pyplot as plt
#import numpy as np

from numpy import True_
import multiprocessing


class PID:
    """ An implementation of a PID control class for use in process control simulations.
    """
    def __init__(self, name=None):
        self.name = name
        self.SP = 0
        self.SP_stepping = 0
        self.SP_increment = 3
        self.SP_range = 0
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0
        self.beta = 0 # 0~1
        self.gamma = 0 # 0~1
        self.MVmin = 0
        self.MVmax = 0
        self.DirectAction = False
        self.mode = False
        self._log = []
        self.errorP0 = 0
        self.errorP1 = 0
        self.errorI0 = 0
        self.errorD0 = 0
        self.errorD1 = 0
        self.errorD2 = 0
        
    def auto(self):
        """Change to automatic control mode. In automatic control mode the .update()
        method computes new values for the manipulated variable using a velocity algorithm.
        """
        #self._mode = 'inAuto'
        self.mode = True
        
    def manual(self):
        """Change to manual control mode. In manual mode the setpoint tracks the process 
        variable to provide bumpless transfer on return to automatic model.
        """
        #self._mode = 'inManual'
        self.mode = False
        
    def _logger(self,t,SP,PV,MV):
        """The PID simulator logs values of time (t), setpoint (SP), process variable (PV),
        and manipulated variable (MV) that can be plotted with the .plot() method.
        """
        self._log.append([t,SP,PV,MV])

    '''
    def plot(self):
        """Create historical plot of SP,PV, and MV using the controller's internal log file.
        """
        dlog = np.asarray(self._log).T
        t,SP,PV,MV = dlog
        plt.subplot(2,1,1)
        plt.plot(t,PV,t,SP)
        plt.title('Process Variable')
        plt.xlabel('Time')
        plt.legend(['PV','SP'])
        plt.subplot(2,1,2)
        plt.plot(t,MV)
        plt.title('Manipulated Variable')
        plt.xlabel('Time')
        plt.tight_layout()
    '''

    @property
    def mode(self):
        return self._mode.value
        
    @mode.setter
    def mode(self,mode):
        if mode:
            self._mode = multiprocessing.Value('i', 1)
        else:
            self._mode = multiprocessing.Value('i', 0)


    @property
    def beta(self):
        """beta is the setpoint weighting for proportional control where the proportional error
        is given by error_proportional = beta*SP - PV. The default value is one.
        """
        return self._beta.value
        
    @beta.setter
    def beta(self,beta):
        self._beta = multiprocessing.Value('d', max(0.0,min(1.0,beta)))
        
    @property
    def DirectAction(self):
        """DirectAction is a logical variable setting the direction of the control. A True
        value means the controller output MV should increase for PV > SP. If False the controller
        is reverse acting, and ouput MV will increase for SP > PV. If the steady state
        process gain is positive then a control will be reverse acting. 
        
        The default value is False.
        """
        return self._DirectAction.value
    
    @DirectAction.setter
    def DirectAction(self,DirectAction):
        if DirectAction:
            self._DirectAction = multiprocessing.Value('i', 1)
            self._action = multiprocessing.Value('i', 1)
        else:
            self._DirectAction = multiprocessing.Value('i', 0)
            self._action = multiprocessing.Value('i', -1)

    @property
    def action(self):
        return self._action.value

    @property
    def gamma(self):
        """gamma is the setpoint weighting for derivative control where the derivative error
        is given by gamma*SP - PV.  The default value is zero. 
        """
        return self._gamma.value
    
    @gamma.setter
    def gamma(self,gamma):
        self._gamma = multiprocessing.Value('d', max(0.0,min(1.0,gamma)))

    @property
    def Kp(self):
        """Kp is the proportional control gain.
        """
        return self._Kp.value
    
    @Kp.setter
    def Kp(self,Kp):
        self._Kp = multiprocessing.Value('d', Kp)
    
    @property
    def Ki(self):
        """Ki is the integral control gain.
        """
        return self._Ki.value
        
    @Ki.setter
    def Ki(self,Ki):
        self._Ki = multiprocessing.Value('d', Ki)
    
    @property
    def Kd(self):
        """Kd is the derivative control gain.
        """
        return self._Kd.value
    
    @Kd.setter
    def Kd(self,Kd):
        self._Kd = multiprocessing.Value('d', Kd)
        
    @property
    def MV(self):
        """MV is the manipulated (or PID outpout) variable. It is automatically
        restricted to the limits given in MVrange.
        """
        return self._MV.value
    
    @MV.setter
    def MV(self,MV):
        self._MV = multiprocessing.Value('d', max(self.MVmin,min(self.MVmax,MV)))
        
    @property
    def MVmin(self):
        return self._MVmin.value
    
    @MVmin.setter
    def MVmin(self, MVmin):
        self._MVmin = multiprocessing.Value('d', MVmin)

    @property
    def MVmax(self):
        return self._MVmax.value
    
    @MVmax.setter
    def MVmax(self, MVmax):
        self._MVmax = multiprocessing.Value('d', MVmax)

    @property
    def SP(self):
        """SP is the setpoint for the measured process variable.
        """
        return self._SP.value
    
    @SP.setter
    def SP(self,SP):
        self._SP = multiprocessing.Value('d', SP)

    @property
    def SP_range(self):
        return self._SP_range.value
    
    @SP_range.setter
    def SP_range(self,SP_range):
        self._SP_range = multiprocessing.Value('d', SP_range)
        
    @property
    def PV(self):
        """PV is the measured process (or control) variable.
        """
        return self._PV.value
    
    @PV.setter
    def PV(self,PV):
        self._PV = multiprocessing.Value('d', PV)

    @property
    def SP_stepping(self):
        return self._SP_stepping.value
    
    @SP_stepping.setter
    def SP_stepping(self,SP_stepping):
        self._SP_stepping = multiprocessing.Value('d', SP_stepping)

    @property
    def SP_increment(self):
        return self._SP_increment.value
    
    @SP_increment.setter
    def SP_increment(self,SP_increment):
        self._SP_increment = multiprocessing.Value('d', SP_increment)
    
    @property
    def errorP0(self):
        return self._errorP0.value
    
    @errorP0.setter
    def errorP0(self,errorP0):
        self._errorP0 = multiprocessing.Value('d', errorP0)

    @property
    def errorP1(self):
        return self._errorP1.value
    
    @errorP1.setter
    def errorP1(self,errorP1):
        self._errorP1 = multiprocessing.Value('d', errorP1)

    @property
    def errorI0(self):
        return self._errorI0.value
    
    @errorI0.setter
    def errorI0(self,errorI0):
        self._errorI0 = multiprocessing.Value('d', errorI0)

    @property
    def errorD0(self):
        return self._errorD0.value
    
    @errorD0.setter
    def errorD0(self,errorD0):
        self._errorD0 = multiprocessing.Value('d', errorD0)

    @property
    def errorD1(self):
        return self._errorD1.value
    
    @errorD1.setter
    def errorD1(self,errorD1):
        self._errorD1 = multiprocessing.Value('d', errorD1)

    @property
    def errorD2(self):
        return self._errorD2.value
    
    @errorD2.setter
    def errorD2(self,errorD2):
        self._errorD2 = multiprocessing.Value('d', errorD2)

    @property
    def kick_prop(self):
        return self._kick_prop.value
    
    @kick_prop.setter
    def kick_prop(self,kick_prop):
        self._kick_prop = multiprocessing.Value('d', kick_prop)
    
    @property
    def tstep(self):
        return self._tstep.value
    
    @tstep.setter
    def tstep(self,tstep):
        self._tstep = multiprocessing.Value('d', tstep)


    def update_paramater(self, Kp=0, Ki=0, Kd=0, beta=0, gamma=0, kick=1, tstep=1, MVmax=0, MVmin=0, SP_range=0, SP_increment=3, DirectAction=0, mode=0,):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.MVmin = MVmin
        self.MVmax = MVmax
        self.beta = beta
        self.gamma = gamma
        self.kick = kick
        self.tstep = tstep
        self.DirectAction = DirectAction
        self.mode = mode
        self.SP_range = SP_range
        self.SP_increment = SP_increment

    def update(self, tstep, SP, PV, MV, kick):
        self.SP = SP
        self.kick_prop = 1
        if self.SP_stepping < self.SP:
            self.SP_stepping += self.SP_increment
            self.kick_prop = kick
            if self.SP_stepping > self.SP:
                self.SP_stepping = self.SP
                self.kick_prop = 1
        elif self.SP_stepping > self.SP:
            self.SP_stepping -= self.SP_increment
            self.kick_prop = kick
            if self.SP_stepping < self.SP:
                self.SP_stepping = self.SP 
                self.kick_prop = 1 
        self.tstep = tstep
        self.PV = PV
        self.MV = MV
        # Setpoint tracking
        if self.mode == 0:
            self.SP_stepping = PV
        self.errorP1 = self.errorP0
        self.errorP0 = self.beta*self.SP_stepping - self.PV # setpoint weighting
        self.errorI0 = self.SP_stepping - self.PV           
        self.errorD2 = self.errorD1
        self.errorD1 = self.errorD0
        self.errorD0 = self.gamma*self.SP_stepping - self.PV # setpoint weighting
        P = self.Kp*(self.errorP0 - self.errorP1)
        I = self.Ki*tstep*self.errorI0
        D = self.Kd*(self.errorD0 - 2*self.errorD1 + self.errorD2)/tstep
        self.deltaMV =  P*self.kick_prop + I + D*self.kick_prop
        if abs(self.SP_stepping - PV) <= self.SP_range:
            self.MV = self.MV
        else:
            self.MV -= self.action*self.deltaMV
        return self.MV, P, I, D