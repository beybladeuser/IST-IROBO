import re
import os
import matplotlib.pyplot as plt
import pandas as pd
import argparse

TOKEN_TO_RM_REGEX = "[\[\],:\(\)]"
def main():
	parser = argparse.ArgumentParser()
	parser.add_argument('-f', '--filename', help='The filename of the log file to make a graph', default='~/.ros/log/latest/error_calc-8-stdout.log')
	parser.add_argument('-o', '--output', help='The ouput name, ignored if filename is a csv', default='./datasets_error')
	parser.add_argument('-t', '--tittle', help='The of tittle the graph', default='Error for dataset with __placeholder__')

	args = parser.parse_args()

	filename = os.path.expanduser(args.filename)
	output:str = ".".join(filename.split(".")[:-1]) if ".csv" in filename else os.path.expanduser(args.output)
	output = "./" + output if len(output.split("/")) == 1 else output

	output_dir = "/".join(output.split("/")[:-1])
	output_filename = output.split("/")[-1]
	index = len(check_similar_filename(output_dir, output_filename))

	with open(filename, "r") as f:
		if ".log" in filename:
			graph_values = parse_log(f)
		elif ".csv" in filename:
			graph_values = parse_csv(f)
		else:
			print("Unsupported file type")

	plt.plot(graph_values["sim_time"], graph_values["error"], marker='')
	plt.title(args.tittle)
	plt.xlabel('Simulation Time')
	plt.ylabel('Error')
	plt.grid()
	
	if ".csv" not in filename:
		graph_values.to_csv(output + f"_{index}.csv")
	plt.savefig(output + f"_{index}.png")

	plt.show()

def check_similar_filename(directory, pattern):
	# List all files in the specified directory
	result = []
	try:
		files = os.listdir(directory)
	except FileNotFoundError:
		print(f"The directory '{directory}' does not exist.")
		return result

	# Compile the regex pattern
	regex = re.compile(pattern)

	# Check if any filename matches the pattern
	for filename in files:
		if regex.match(filename) and ".csv" not in filename:
			result.append(filename)

	return result

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