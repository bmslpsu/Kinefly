#!/usr/bin/env python
from __future__ import division
import Phidgets
import Phidgets.Devices.Analog


class Flystate2PhidgetsAnalog:

    def __init__(self):
        self.bInitialized = False
        self.iCount = 0
        self.bAttached = False

        # Connect to the Phidget.
        self.analog = Phidgets.Devices.Analog.Analog()
        self.analog.openPhidget()
        # if self.params['serial'] == 0:
        #     self.analog.openPhidget()
        # else:
        #     self.analog.openPhidget(self.params['serial'])

        self.analog.setOnAttachHandler(self.attach_callback)
        self.analog.setOnDetachHandler(self.detach_callback)

        self.iCount = 0
        self.vCount = 0

        self.bInitialized = True

    def attach_callback(self, phidget):
        for i in range(4):
            self.analog.setEnabled(i, self.enable[i])

        self.phidgetserial = self.analog.getSerialNum()
        self.phidgetname = self.analog.getDeviceName()
        print('%s - %s Attached: serial=%s' % (self.namespace, self.phidgetname, self.phidgetserial))
        self.bAttached = True
        print('**************************************************************************')

    def detach_callback(self, phidget):
        print('%s - %s Detached: serial=%s.' % (self.namespace, self.phidgetname, self.phidgetserial))
        self.bAttached = False
        print('**************************************************************************')

    def run(self):
        while self.analog.isAttached():
            for i in range(4):
                self.analog.setVoltage(i, 5)

        self.analog.closePhidget()


if __name__ == '__main__':
    s2pa = Flystate2PhidgetsAnalog()
    s2pa.run()