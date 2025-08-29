from cProfile import label
from pkgutil import get_data
import matplotlib.pyplot as plt 
from matplotlib.lines import Line2D
#import matplotlib.animation as animation
import numpy as np

plt.ion()  # enable interactive drawing


class dataPlotter:
    ''' 
        This class plots the time histories for the usv data.
    '''

    def __init__(self):
        # Number of subplots = num_of_rows*num_of_cols
        self.num_rows = 2    # Number of subplot rows
        self.num_cols = 1    # Number of subplot columns

        # Crete figure and axes handles
        self.fig, self.ax = plt.subplots(self.num_rows, self.num_cols, sharex=True)

        # Instantiate lists to hold the time and data histories
        self.time_history = []  # time
        self.theta_ref_history = []  # reference angle
        self.theta_history = []  # angle theta
        self.ctrl_l_history = []  # control T_l
        self.ctrl_r_history = []  # control T_r

        # create a handle for every subplot.
        self.handle = []
        self.handle.append(myPlot(self.ax[0], ylabel='theta(deg)', title='usv Data',legend=["act","ref"])) #,legend=("act psi","ref psi")
        self.handle.append(myPlot(self.ax[1], xlabel='t(s)', ylabel='T_lb&T_rg(N)',legend=["T_l","T_r"])) #,legend=("T_L","T_r")

    def update(self, t, reference, state, ctrl):
        '''
            Add to the time and data histories, and update the plots.
        '''
        # update the time history of all plot variables
        self.time_history.append(t)  # time
        self.theta_ref_history.append(reference)  # reference base position
        #self.theta_history.append(180.0/np.pi*states.item(0))  # rod angle (converted to degrees)
        self.theta_history.append(state)  # angle (degrees)
        self.ctrl_l_history.append(ctrl[0])  # force on the base
        self.ctrl_r_history.append(ctrl[1])  # force on the base

        # update the plots with associated histories
        self.handle[0].update(self.time_history, [self.theta_history, self.theta_ref_history])
        #self.handle[1].update(self.time_history, [self.torque_history])
        self.handle[1].update(self.time_history, [self.ctrl_l_history,self.ctrl_r_history])


class myPlot:
    ''' 
        Create each individual subplot.
    '''
    def __init__(self, ax,
                 xlabel='',
                 ylabel='',
                 title='',
                 legend=None):
        ''' 
            ax - This is a handle to the  axes of the figure
            xlable - Label of the x-axis
            ylable - Label of the y-axis
            title - Plot title
            legend - A tuple of strings that identify the data. 
                     EX: ("data1","data2", ... , "dataN")
        '''
        self.legend = legend
        self.ax = ax                  # Axes handle
        self.colors = ['b', 'g', 'r', 'c', 'm', 'y', 'b']
        # A list of colors. The first color in the list corresponds
        # to the first line object, etc.
        # 'b' - blue, 'g' - green, 'r' - red, 'c' - cyan, 'm' - magenta
        # 'y' - yellow, 'k' - black
        self.line_styles = ['-', '-', '--', '-.', ':']
        # A list of line styles.  The first line style in the list
        # corresponds to the first line object.
        # '-' solid, '--' dashed, '-.' dash_dot, ':' dotted

        self.line = []

        # Configure the axes
        self.ax.set_ylabel(ylabel)
        self.ax.set_xlabel(xlabel)
        self.ax.set_title(title)
        self.ax.grid(True)

        # Keeps track of initialization
        self.init = True   

    def update(self, time, data):
        ''' 
            Adds data to the plot.  
            time is a list, 
            data is a list of lists, each list corresponding to a line on the plot
        '''
        if self.init == True:  # Initialize the plot the first time routine is called
            for i in range(len(data)):
                # Instantiate line object and add it to the axes
                self.line.append(Line2D(time,
                                        data[i],
                                        color=self.colors[np.mod(i, len(self.colors) - 1)],
                                        ls=self.line_styles[np.mod(i, len(self.line_styles) - 1)],
                                        label=self.legend[i] if self.legend != None else None))
                self.ax.add_line(self.line[i])
            self.init = False
            # add legend if one is specified
            if self.legend != None:
                plt.legend(handles=self.line)
        else: # Add new data to the plot
            # Updates the x and y data of each line.
            for i in range(len(self.line)):
                self.line[i].set_xdata(time)
                self.line[i].set_ydata(data[i])

        # Adjusts the axis to fit all of the data
        self.ax.relim()
        self.ax.autoscale()
        plt.draw()
           
class trajectoryDrawer():
    def __init__(self,x_list,y_list):
        self.x_list = x_list
        self.y_list = y_list
        #self.text_pt = plt.text(3,5,0.8,'',fontsize=16)

    
    def update(self,i):
        point_ani.set_data(self.x_list[i],self.y_list[i])
        #self.text_pt.set_text("x=%.2f, y=%.2f"%(self.x_list[i],self.y_list[i]))
        return self.point_ani
    
    def trajectoryShow(self):
        fig = plt.figure(tight_layout=True)
        point_ani, = plt.plot(self.x_list[0],self.y_list[0],"r-")
        ani = animation.FuncAnimation(fig=fig, frames=np.arange(len(self.x_list)), func=self.update, interval=10,blit=True)
        plt.show()

