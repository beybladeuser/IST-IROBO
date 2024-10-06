import re
import os

token_to_rm_regex = "[\[\],:\(\)]"
graph_values = []
start_time = None
with open(os.path.expanduser("~/.ros/log/latest/error_calc-8-stdout.log"), "r") as f:
	for line in f:
		line = re.sub(token_to_rm_regex, "", line.strip())
		line_values = line.split(" ")
		if "error" != line_values[3].lower():
			continue
		
		if start_time == None:
			start_time = float(line_values[2])
		temp = {
			"x": float(line_values[2]) - start_time, #sim_time
			"y": float(line_values[6]), #error
		}
		graph_values.append(temp)
