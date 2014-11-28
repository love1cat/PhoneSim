#!/Users/andy/Tools/gnuplot/bin/gnuplot

###### Plots all type costs for all algorithms ######
set style data histogram
set style histogram cluster gap 1
set style fill solid border -1
set boxwidth 0.9
set xtic scale 0
set xtics ("MinSense" 0, "FairSense" 1, "Estimation-based" 2, "Adaptive cost" 3, "Baseline" 4)
set xtic rotate by -45 scale 0
set key left top

dcost_log = "costs_result.txt"
dcost_eps = "diff_cost.eps"

#set yrange [0.14:0.47]
set title "Average cost of different phone activities"
SENSING = "#99ffff" 
COMM = "#4671d5" 
UPLOAD = "#ff0000"
set auto x
#set yrange [0:10]

set terminal postscript eps color enhanced "Times-Roman" 26
set output dcost_eps
plot dcost_log using 1 ti "Data sensing",  \
     '' u 2 ti "Data exchange", \
     '' u 3 ti "Data offloading"
