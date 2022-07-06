#import matplotlib.pyplot as plt
#import numpy as np

from numpy import True_


class PID:
    """ An implementation of a PID control class for use in process control simulations.
    """
    def __init__(self, name=None):
        self.name = name
        self.SP = None
        self.Kp = None
        self.Ki = None
        self.Kd = None
        self.beta = 0 # 0~1
        self.gamma = 0 # 0~1
        self.MVmin = 0
        self.MVmax = 0
        self.DirectAction = False
        self.mode = False
        self._log = []
        self._errorP0 = 0
        self._errorD0 = 0
        self._errorD1 = 0
        
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
    def beta(self):
        """beta is the setpoint weighting for proportional control where the proportional error
        is given by error_proportional = beta*SP - PV. The default value is one.
        """
        return self._beta
        
    @beta.setter
    def beta(self,beta):
        self._beta = max(0.0,min(1.0,beta))
        
    @property
    def DirectAction(self):
        """DirectAction is a logical variable setting the direction of the control. A True
        value means the controller output MV should increase for PV > SP. If False the controller
        is reverse acting, and ouput MV will increase for SP > PV. If the steady state
        process gain is positive then a control will be reverse acting. 
        
        The default value is False.
        """
        return self._DirectAction
    
    @DirectAction.setter
    def DirectAction(self,DirectAction):
        if DirectAction:
            self._DirectAction = True
            self._action = +1.0
        else:
            self._DirectAction = False
            self._action = -1.0
    
    @property
    def gamma(self):
        """gamma is the setpoint weighting for derivative control where the derivative error
        is given by gamma*SP - PV.  The default value is zero. 
        """
        return self._gamma
    
    @gamma.setter
    def gamma(self,gamma):
        self._gamma = max(0.0,min(1.0,gamma))
    
    @property
    def Kp(self):
        """Kp is the proportional control gain.
        """
        return self._Kp
    
    @Kp.setter
    def Kp(self,Kp):
        self._Kp = Kp
    
    @property
    def Ki(self):
        """Ki is the integral control gain.
        """
        return self._Ki
        
    @Ki.setter
    def Ki(self,Ki):
        self._Ki = Ki
    
    @property
    def Kd(self):
        """Kd is the derivative control gain.
        """
        return self._Kd
    
    @Kd.setter
    def Kd(self,Kd):
        self._Kd = Kd
        
    @property
    def MV(self):
        """MV is the manipulated (or PID outpout) variable. It is automatically
        restricted to the limits given in MVrange.
        """
        return self._MV
    
    @MV.setter
    def MV(self,MV):
        self._MV = max(self._MVmin,min(self._MVmax,MV))
        
    @property
    def MVmin(self):
        return self._MVmin
    
    @MVmin.setter
    def MVmin(self, MVmin):
        self._MVmin = MVmin

    @property
    def MVmax(self):
        return self._MVmax
    
    @MVmax.setter
    def MVmax(self, MVmax):
        self._MVmax = MVmax

    @property
    def SP(self):
        """SP is the setpoint for the measured process variable.
        """
        return self._SP
    
    @SP.setter
    def SP(self,SP):
        self._SP = SP
        
    @property
    def PV(self):
        """PV is the measured process (or control) variable.
        """
        return self._PV
    
    @PV.setter
    def PV(self,PV):
        self._PV = PV

    def update_paramater(self, Kp=0, Ki=0, Kd=0, beta=0, gamma=0, MVmin=0, MVmax=0, DirectAction=False, mode=False):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Kd = Kd
        self.MVmin = MVmin
        self.MVmax = MVmax
        self.beta = beta
        self.gamma = gamma
        self.DirectAction = DirectAction
        self.mode = mode

    def update(self, tstep, SP, PV, MV):
        self.SP = SP
        self.PV = PV
        self.MV = MV # MV tracking

        if self.mode == False:
            # Setpoint tracking
            self.SP = PV 

        self._errorP1 = self._errorP0
        self._errorP0 = self.beta*self.SP - self.PV # setpoint weighting
        self._errorI0 = self.SP - self.PV            
        self._errorD2 = self._errorD1
        self._errorD1 = self._errorD0
        self._errorD0 = self.gamma*self.SP - self.PV # setpoint weighting
        # if self.mode == True:
        P = self.Kp*(self._errorP0 - self._errorP1)
        I = self.Ki*tstep*self._errorI0
        D = self.Kd*(self._errorD0 - 2*self._errorD1 + self._errorD2)/tstep
        self._deltaMV =  P + I + D
        print(self._deltaMV)
        print(self.MV)
        self.MV -= self._action*self._deltaMV
        print(self.MV)
        #self._logger(self.SP,self.PV,self.MV)
        
        return self.MV, P, I, D