#!/usr/bin/python3

"""ファイルの最大値・最小値・平均値・標準偏差を表示する"""

import argparse
import sys
import statistics
import pathlib

import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("file", type=open, help="make statics of the file")
args = parser.parse_args()

figfile = pathlib.PurePath(args.file.name).stem + ".png"

# skip first 100 lines
for i in range(100):
    null = args.file.readline()

nums = [float(n) for n in args.file.readlines()]
if len(nums) < 100000:
    print("data sample is less than 100000. exit.")
    sys.exit(1)
nums = nums[0:100000]

strmin = "{:.1f}".format(min(nums))
strmax = "{:.1f}".format(max(nums))
stravg = "{:.1f}".format(statistics.mean(nums))
strsd = "{:.1f}".format(statistics.stdev(nums))

print("- Min: ", strmin, "us")
print("- Max: ", strmax, "us")
print("- Mean:", stravg, "us")
print("- SD:  ", strsd)

xdata = [i for i in range(0, 100000)]
plt.plot(nums, "r.", markersize=1)
plt.title("Min: " + strmin + "us, " +
          "Max: " + strmax + "us, " +
          "Mean: " + stravg + "us, " +
          "SD: " + strsd)
plt.xlabel("Count (1000Hz)")
plt.ylabel("Latency (us)")
plt.ylim(0.0, 5000.0)
plt.savefig(figfile, dpi=300)
