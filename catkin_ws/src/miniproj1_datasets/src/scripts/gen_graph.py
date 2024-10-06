import re
import os

token_to_rm_regex = "[\[\],:\(\)]"
graph_values = []
with open(os.path.expanduser("~/.ros/log/latest/error_calc-8-stdout.log"), "r") as f:
	for line in f:
		line = re.sub(token_to_rm_regex, "", line.strip())
		if "error" not in line.lower():
			continue
		line_values = line.split(" ")

		temp = {
			"x": line_values[2], #sim_time
			"y": line_values[6], #error
		}
		print(temp)
		graph_values.append(temp)
