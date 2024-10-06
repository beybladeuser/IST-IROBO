import re
import os
import matplotlib.pyplot as plt
import numpy as np

filename = "~/.ros/log/latest/error_calc-8-stdout.log"
#filename = "temp.txt"
token_to_rm_regex = "[\[\],:\(\)]"
graph_values = []
start_time = None
with open(os.path.expanduser(filename), "r") as f:
	for line in f:
		line = re.sub(token_to_rm_regex, "", line.strip())
		line_values = line.split(" ")
		if "error" != line_values[3].lower():
			continue
		
		if start_time == None:
			start_time = float(line_values[2])

		sim_time = float(line_values[2]) - start_time
		error = float(line_values[6])

		graph_values.append((sim_time, error))

graph_values = np.array(graph_values)

plt.plot(graph_values[:, 0], graph_values[:, 1], marker='o')
plt.title('Simple Line Graph')
plt.xlabel('Simulation Time')
plt.ylabel('Error')
plt.grid()
plt.show()