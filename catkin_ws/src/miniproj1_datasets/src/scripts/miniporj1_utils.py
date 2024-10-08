import numpy as np
import re
import os

def split_extension_from_filename(filename:str):
	filename_components = np.array(filename.split("."))
	filename = ".".join(filename_components[:-1])
	extension = filename_components[-1]
	if "/" in extension:
		filename = ".".join(filename, extension)
		extension = None
	return filename, extension

def split_path_from_filename(filename:str):
	filename_components = np.array(filename.split("/"))
	path = "/".join(filename_components[:-1])
	filename = filename_components[-1]
	return path, filename

def infer_current_dir(filename:str):
	return "./" + filename if re.match(r'^[a-zA-Z0-9].*', filename) != None else filename

def check_similar_filename(directory, pattern):
	# List all files in the specified directory
	result = []
	try:
		files = os.listdir(directory)
	except FileNotFoundError:
		print(f"The directory '{directory}' does not exist.")
		return result

	# Compile the regex pattern
	regex = re.compile(pattern + r"_[0-9]+\.png$")

	# Check if any filename matches the pattern
	for filename in files:
		if regex.match(filename):
			result.append(filename)

	return result

def get_dup_file_index(filename, has_extension=True, index_mod=0):
	if has_extension:
		filename = split_extension_from_filename(filename)[0]
	path, filename = split_path_from_filename(filename)
	return len(check_similar_filename(path, filename)) + index_mod

def expand_filename(filename):
	return infer_current_dir(os.path.expanduser(filename))

def create_file(filename):
	filename = expand_filename(filename)
	path, filename = split_path_from_filename(filename)
	dirs = np.array(path.split("/"))
	for i in range(1,len(dirs)):
		current_path = "/".join(dirs[:i+1])
		if not os.path.exists(current_path):
			os.mkdir(current_path)