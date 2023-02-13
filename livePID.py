import matplotlib.pyplot as plt
import numpy as np

max_length = 16000

fig = plt.figure()
ax = fig.add_subplot(111)


xaxis, yaxis = np.zeros(max_length), np.zeros(max_length)

accel_interval = 0.005
max_step_delay = 5000
min_step_delay = 100

def generateStepArrays(num_steps): 

    accel = 0
    accel_steps = 0
    decel_steps = 0
    accel_check = True
    accel_interval = 0.0005

    for i in range(num_steps):

        xaxis[i] = i

        if yaxis[i] > max_step_delay - (max_step_delay - min_step_delay)/2 and accel_check == True:

            accel = accel + accel_interval

            yaxis[i+1] = yaxis[i] + accel


def findAccelInterval(accel_interval, desired_steps):

    step_count = 0
    accel = 0
    yaxis[0] = max_step_delay
    interval_found = False

    while (interval_found == False):

        for i in range(desired_steps):

            xaxis[i] = i

            if yaxis[i] > max_step_delay - (max_step_delay - min_step_delay)/2: 

                accel = accel + accel_interval

                yaxis[i+1] = yaxis[i] - accel

            elif yaxis[i] < max_step_delay - (max_step_delay - min_step_delay)/2 and accel > 0:

                accel = accel - accel_interval

                yaxis[i+1] = yaxis[i] - accel

            else:

                yaxis[i] = min_step_delay

                #print("done")

        if yaxis[desired_steps-1] != min_step_delay:

            #print(yaxis[desired_steps])

            accel_interval = accel_interval + 0.001

            #print(accel_interval)
            
        else:

            interval_found = True

    return accel_interval

required_interval = findAccelInterval(accel_interval, 8000)

print(required_interval)

plt.plot(xaxis,yaxis)
plt.show()
      
    

