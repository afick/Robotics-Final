# Author: Carter Kruse
# Date: May 18, 2023

# Import Relevant Libraries (Python Modules)
import csv

results = []

# Read the CSV file into an array.
with open("data.csv") as file:
    reader = csv.reader(file, quoting = csv.QUOTE_NONNUMERIC) # Constants - Floats
    for row in reader:
        results.append(row)

# Redefine the elements of the array.
for i in range(len(results)):
    for j in range(len(results[0])):
        if results[i][j] == -1:
            results[i][j] = -100

# Print the results.
print(results)