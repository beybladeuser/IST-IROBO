import re
import os
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import argparse
import miniporj1_utils as utils

TOKEN_TO_RM_REGEX = "[\[\],:\(\)]"
def main():
	parser = argparse.ArgumentParser()
	parser.add_argument('-f', '--filename', help='The filename of the log file to make a graph', default='~/.ros/log/latest/error_calc-8-stdout.log')
	parser.add_argument('-o', '--output', help='The ouput name, ignored if filename is a csv', default='./datasets_error')
	parser.add_argument('-t', '--title', help='The of title the graph', default='Error for dataset with __placeholder__')

	args = parser.parse_args()

	filename = os.path.expanduser(args.filename)
	filename_extensionless, filename_extension = utils.split_extension_from_filename(filename)
	output:str = utils.infer_current_dir(filename_extensionless if "csv" == filename_extension else os.path.expanduser(args.output))

	index = utils.get_dup_file_index(output, has_extension=False, index_mod = 1 if "csv" == filename_extension else 0)

	with open(filename, "r") as f:
		if ".log" in filename:
			graph_values = parse_log(f)
		elif ".csv" in filename:
			graph_values = parse_csv(f)
		else:
			print("Unsupported file type")

	plt.plot(np.array(graph_values["sim_time"]), np.array(graph_values["error"]), marker='')
	plt.title(args.title)
	plt.xlabel('Simulation Time')
	plt.ylabel('Error')
	plt.grid()
	
	if ".csv" not in filename:
		graph_values.to_csv(output + f"_{index}.csv")
	plt.savefig(output + f"_{index}.png")

	plt.show()

def parse_csv(file) -> pd.DataFrame:
	return pd.read_csv(file, index_col="id")

def parse_log(file) -> pd.DataFrame:
	graph_values = []
	start_time = None
	for line in file:
		line = re.sub(TOKEN_TO_RM_REGEX, "", line.strip())
		line_values = line.split(" ")
		if "error" != line_values[3].lower():
			continue
		
		if start_time == None:
			start_time = float(line_values[2])

		sim_time = float(line_values[2]) - start_time
		error = float(line_values[6])

		temp = {
			"sim_time": sim_time,
			"error": error
		}

		graph_values.append(temp)
	
	return pd.DataFrame(graph_values).rename_axis("id")

main()