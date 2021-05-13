#!/usr/bin/env python
# coding=utf-8

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
class Environment(object):
    def __init__(self,init_wing_gain=0, init_head_gain=0, init_arena_gain=5):
        self.bInitialized = False

        # Initialize node
        self.name = 'environment'
        rospy.init_node(self.name, anonymous=True)
        self.params = rospy.get_param(self.name, {})
        # rospy.set_param('%s' % self.name, self.params)

        # Create panels control object & set the pattern ID and initial position
        self.panel_control = LEDPanels()
        self.panel_control.stop()
        self.panel_control.set_pattern_id(self.params['panels']['pattern_id'])
        self.panel_control.set_position(self.params['panels']['pattern_xpos'],
                                        self.params['panels']['pattern_ypos'])
        self.panel_control.send_gain_bias_16(init_arena_gain,0,0,0)
        self.panel_control.set_mode(1, 0) # closed-loop

        # Set up variable to store panel data
        self.panelCount = 0
        self.panel_hist_size = 500000
        self.panel_hist = np.empty((self.panel_hist_size, 1))
        self.panel_hist[:] = np.NaN

        # Create dynamic reconfigure cliet to control Phidget gains
        self.phidget_nodename = 'kinefly/flystate2phidgetsanalog'
        self.phidget_client = dynamic_reconfigure.client.Client(self.phidget_nodename)
        self.phidget_params = rospy.get_param(self.phidget_nodename, {}) # get the Phidget parameters
        self.phidget_config = [] # to store phidget configuration
        self.setGains(wing_gain=init_wing_gain, head_gain=init_head_gain) # start with 0 gains

        # Subscribe to the DAQ message topic to get panels position
        self.subPanelstate = rospy.Subscriber('/mcdaq/AI', MC_AnalogIN, self.panelstate_callback, queue_size=1000)

        self.bInitialized = True
        rospy.logwarn(' Environment initialized')
        # rospy.spin()

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
        panel_data = np.rad2deg(np.unwrap(np.deg2rad(panel_data), axis=0))
        time_vector = np.linspace(0, time_period, panel_data.shape[0])

        if showplot:
            plt.close('all')
            fig, ax = plt.subplots()
            ax.plot(time_vector, panel_data[:,0])
            plt.show(False)

        return panel_data, time_vector

    def setGains(self, wing_gain=0, head_gain=0):
        self.wing_gain = wing_gain
        self.head_gain = head_gain
        new_params = {self.params['phidget']['lwing_channel']:  wing_gain,
                      self.params['phidget']['rwing_channel']: -wing_gain,
                      self.params['phidget']['head_channel']:   head_gain}
        self.phidget_config = self.phidget_client.update_configuration(new_params)

    def setFunc(self, funcID=0):
        self.panel_control.stop()
        self.panel_control.set_position(self.params['panels']['pattern_xpos'],
                                        self.params['panels']['pattern_ypos'])
        self.panel_control.send_gain_bias_16(0,0,0,0)
        self.panel_control.set_mode(4, 0) # position funciton mode
        self.panel_control.set_funcx_freq(400)
        self.panel_control.set_posfunc_id(1, funcID)

    def act(self, action_time=5, rest_time=2):
        # Stop panels & reset bar position
        self.panel_control.stop()
        self.panel_control.set_position(self.params['panels']['pattern_xpos'],
                                        self.params['panels']['pattern_ypos'])
        time.sleep(0.5) # wait to start panels

        # Update gains
        self.setGains(wing_gain=5, head_gain=0)

        # Start panels
        self.panel_control.start()

        # Wait for fly acclimate before to start data collection
        time.sleep(rest_time)

        # Collect panel response
        panel_data, time_vector = self.getpaneldata(action_time, showplot=False)
        panel_data = panel_data - panel_data[0] # normalize to start at 0

        # Stop panels & reset bar position
        self.panel_control.stop()
        self.panel_control.set_position(self.params['panels']['pattern_xpos'],
                                        self.params['panels']['pattern_ypos'])

        # Compute reward
        reward = self.rewardfrompanel(panel_data, action_time)

        return reward, panel_data, time_vector

    def rewardfrompanel(self, panel_data, episode_time):
        # Get the cost/reward from the bar trajectory
        # print('Panel length: ' + str(len(panel_data)))
        # dc = np.mean(panel_data)
        dc = 0
        error = panel_data - dc
        cost = np.sum(error**2) / (len(panel_data) * episode_time)
        # reward = 1 / cost
        reward = -cost

        return reward


if __name__ == '__main__':
    env = Environment(init_wing_gain=1, init_head_gain=0, init_arena_gain=0)
    env.act()
    time.sleep(5)

    # self.panel_control.stop()
    # self.panel_control.set_position(1,self.panel_ypos)
    # self.panel_control.set_mode(4, 0)
    # self.panel_control.set_posfunc_id(1, 5)
    # self.panel_control.set_funcx_freq(400)
    # self.panel_control.start()

    rospy.spin()