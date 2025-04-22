
import os

# Get filesystem stats
stat = os.statvfs("/")

# Calculate total space
total_space = stat[0] * stat[2] # Block size * Total blocks

# Calculate free space
free_space = stat[0] * stat[3] # Block size * Free blocks

# Print results in bytes
print(f"Total space: {total_space} Bytes")
print(f"Free space:  {free_space} Bytes")

