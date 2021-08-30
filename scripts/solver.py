#! /usr/bin/env python3
import os
import time
import subprocess


INPUT_FILE_PATH = "piecesVertices.csv"
RESULTS_FILE_PATH = "results.csv"
RESULTS_R_FILE_PATH = "resR"

def wait_for_input_file():
	while not os.path.exists(INPUT_FILE_PATH):
		print("waiting for file...")
		time.sleep(5)


def run_solver():
	subprocess.call(
		[r"/home/jigsaw/Desktop/thesis/solver/build/bin/physics", "-p", INPUT_FILE_PATH, "-t", "0.7", "-P", RESULTS_FILE_PATH, "-R", RESULTS_R_FILE_PATH])


if os.path.exists(INPUT_FILE_PATH):
	choice = input("Input file already in folder. Do you want to run solver on this file? [Y/n] ")
	if choice.lower() != "y":  # if user asked not to run solver on file so deleting and waiting for new file
		os.remove(INPUT_FILE_PATH)
		os.remove(RESULTS_FILE_PATH)

while True:
	wait_for_input_file()
	run_solver()
	time.sleep(40)
	os.remove(INPUT_FILE_PATH)
	os.remove(RESULTS_FILE_PATH) 
	os.remove(RESULTS_R_FILE_PATH)

