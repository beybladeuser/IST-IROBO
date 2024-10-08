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

	filename = utils.expand_filename(args.filename)
	filename_extensionless, filename_extension = utils.split_extension_from_filename(filename)
	output:str = filename_extensionless if "csv" == filename_extension else utils.expand_filename(args.output)

	index = utils.get_dup_file_index(output, has_extension=False, index_mod = 1 if "csv" == filename_extension else 0)

	print("Opening file " + filename)
	with open(filename, "r") as f:
		if ".log" in filename:
			print("Parsing as log file")
			graph_values = parse_log(f)
		elif ".csv" in filename:
			print("Parsing as csv file")
			graph_values = parse_csv(f)
		else:
			print("Unsupported file type")

	print("Creating plot")
	plt.plot(np.array(graph_values["sim_time"]), np.array(graph_values["error"]), marker='')
	plt.title(args.title)
	plt.xlabel('Simulation Time')
	plt.ylabel('Error')
	plt.grid()
	
	if "csv" != filename_extension:
		utils.create_file(filename)
		print("Saving data as csv into " + output + f"_{index}.csv")
		graph_values.to_csv(output + f"_{index}.csv")

	print("Saving plot as png into " + output + f"_{index}.png")
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