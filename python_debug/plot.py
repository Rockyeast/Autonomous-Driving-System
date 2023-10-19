import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

'''
# st
data = pd.read_csv("/home/pluto/VSCode_Workspace/em_planner/modules/Debug/log/data.csv")
plt.plot(data["lt"], data["ls"], color='blue', linestyle='-', label='down')
plt.plot(data["ut"], data["us"], color='red', linestyle='--', label='up')
for i in range(len(data["lt"])):
    plt.annotate((data["lt"][i], data["ls"][i]), (data["lt"][i], data["ls"][i]-2))
for i in range(len(data["ut"])):
    plt.annotate((data["ut"][i], data["us"][i]), (data["ut"][i], data["us"][i]+2))
plt.legend()
plt.ylim(-1,200)
plt.xlim(-1,9)
plt.xlabel('T')
plt.ylabel('S')
plt.title('ST')
plt.show()
'''

# trj
obs_data = pd.read_csv("/home/pluto/VSCode_Workspace/em_planner/modules/Debug/log/obs_trj.csv")
adc_data = pd.read_csv("/home/pluto/VSCode_Workspace/em_planner/modules/Debug/log/adc_path.csv")
plt.plot(obs_data["x"], obs_data["y"], color='yellow', linestyle='-', label='obs')
plt.plot(adc_data["x"], adc_data["y"], color='blue', linestyle='--', label='adc')
for obs in obs_data["x"]:
  for adc in adc_data["x"]:
    if obs==adc:
      plt.plot(obs, 0, color='pink', linestyle=':', label='overlap')
      plt.annotate(obs, (obs, 0))
    	

plt.legend()
plt.xlim(-1,200)
plt.ylim(-1,1)

plt.title('XY')
plt.show()


 



