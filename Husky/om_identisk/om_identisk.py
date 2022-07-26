from asyncore import read
import pandas as pd

# Read the files lines and put them in a list
file1 = open('før.txt', 'r')
file2 = open('etter.txt', 'r')
lines1 = file1.readlines()
lines2 = file2.readlines()
file1.close()
file2.close()

# list of unique that are in file2 but not in file1
unique_in_file2 = []
for line in lines2:
    if line not in lines1:
        unique_in_file2.append(line)

# remove empty lines
unique_in_file2 = [x for x in unique_in_file2 if x]

# Print the unique lines
print('Unique lines in file2 but not in file1:')
for line in unique_in_file2:
    print(line)

# Write to result file
file3 = open('resultat.txt', 'w')
for line in unique_in_file2:
    file3.write(line)
file3.close()