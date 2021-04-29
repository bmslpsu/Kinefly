#!/usr/bin/env python

from __future__ import division
import rospy
import dynamic_reconfigure.client

import numpy as np
import time
import matplotlib.pyplot as plt

from mcdaq.msg import MC_AnalogIN
from ledpanelsdirect_noros import LEDPanels

###############################################################################
###############################################################################
#
#
class LogisticPolicy(object):
    def __init__(self):
        self.bInitialized = False

        # Initialize node
        self.name = 'RL'
        rospy.init_node(self.name, anonymous=True)
        self.nodename = rospy.get_name()
        self.namespace = rospy.get_namespace()
        rospy.logwarn(self.nodename)
        self.params = rospy.get_param(self.name, {})
        rospy.set_param('%s' % self.name, self.params)
        self.params = self.params['RL']

        # Create panels control object & set the pattern ID and initial position
        self.panel_control = LEDPanels()
        self.panel_control.stop()
        self.panel_control.set_pattern_id(self.params['panels']['pattern_id'])
        self.panel_control.set_position(self.params['panels']['pattern_xpos'],
                                        self.params['panels']['pattern_ypos'])
        self.panel_control.set_mode(1, 0) # closed-loop

        # Setup variable to store panel data
        self.panelCount = 0
        self.panel_hist_size = 500000
        self.panel_hist = np.empty((self.panel_hist_size, 1))
        self.panel_hist[:] = np.NaN

        # Create dynamic reconfigure cliet to control Phidget gains
        self.phidget_nodename = 'kinefly/flystate2phidgetsanalog'
        self.phidget_client = dynamic_reconfigure.client.Client(self.phidget_nodename)
        self.phidget_params = rospy.get_param(self.phidget_nodename, {}) # get the Phidget parameters
        self.phidget_config = [] # to store phidget configuration
        self.setGains(wing_gain=0, head_gain=0) # start with 0 gains

        # Subscribe to the DAQ message topic to get panels position
        self.subPanelstate = rospy.Subscriber('/mcdaq/AI', MC_AnalogIN, self.panelstate_callback, queue_size=1000)

        # Set learning hyperparameters
        self.episode_time = 5
        self.rest_time = 1

        self.bInitialized = True
        rospy.logwarn('Initialized')

    def panelstate_callback(self, panelstate):
        if self.bInitialized:
            if self.panelCount > self.panel_hist_size: # reset counter if left on for too long
                rospy.logwarn('Panel buffer size exceeded, starting over')
                self.panelCount = 0
                self.panel_hist = np.empty((self.panel_hist_size, 1))
                self.panel_hist[:] = np.NaN

            pattern_pos = self.panelvoltage2deg(panelstate.voltages[0])
            self.panel_hist[self.panelCount, 0] = pattern_pos
            self.panelCount += 1 # increment panel data index

    def panelvoltage2deg(self,volt):
        pixel = np.round(96*(volt/10), 0)
        deg = 3.75 * pixel
        return deg

    def getpaneldata(self, time_period, showplot=False):
        # Clear the panel history que and collect new data for set time

        # Reset panel history
        self.panelCount = 0 # reset panel count
        self.panel_hist = np.empty((self.panel_hist_size, 1)) # reset panel history
        self.panel_hist[:] = np.NaN
        time.sleep(time_period) # wait for one episode length

        # Pull out panel data for current period
        nanI = np.isnan(self.panel_hist[:,0])
        lastI = np.argmax(nanI)
        panel_data = self.panel_hist[0:lastI,:]
        # tt = np.arange(0,panel_data.shape[0])
        # tt = tt.reshape(-1,1)
        tt = np.linspace(0,time_period,panel_data.shape[0])

        if showplot:
            plt.close('all')
            fig, ax = plt.subplots()
            line1, = ax.plot(tt, panel_data[:,0])
            plt.show(False)

        return panel_data

    def setGains(self, wing_gain=0, head_gain=0):
        new_params = {self.params['phidget']['lwing_channel']:  wing_gain,
                      self.params['phidget']['rwing_channel']: -wing_gain,
                      self.params['phidget']['head_channel']:  head_gain}
        self.phidget_config = self.phidget_client.update_configuration(new_params)

    def act(self):

        # Stop panels & reset bar position
        self.panel_control.stop()
        self.panel_control.set_position(self.params['panels']['pattern_xpos'],
                                        self.params['panels']['pattern_ypos'])

        # Update gains
        self.setGains(wing_gain=5, head_gain=0)

        # Start panels
        self.panel_control.start()

        # Wait for fly accilmate before to start data collection
        time.sleep(self.rest_time)

        # Collect panel response
        panel_data = self.getpaneldata(self.episode_time, showplot=False)

        # Compute reward
        reward = self.costfrompanel(panel_data)


    def costfrompanel(self, panel_data):
        # Get the cost from bar trajetcory in degrees
        cost = np.sum(panel_data)

        return cost


if __name__ == '__main__':
    Agent = LogisticPolicy()

    Agent.act()

    time.sleep(5)

    # Agent.act()
    #
    # time.sleep(3)
    #
    # Agent.act()

    # self.panel_control.stop()
    # self.panel_control.set_position(1,self.panel_ypos)
    # self.panel_control.set_mode(4, 0)
    # self.panel_control.set_posfunc_id(1, 5)
    # self.panel_control.set_funcx_freq(400)
    # self.panel_control.start()

    rospy.spin()