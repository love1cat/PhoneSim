#!/Users/andy/Tools/gnuplot/bin/gnuplot

# set style
lwidth = 2
psize =  4 
fontsize = 26
set autoscale
set style data linespoints
set key left top
set style line 1 lt 1 lw lwidth pt 3 ps psize 
set style line 2 lt 3 lw lwidth pt 4 ps psize
set style line 3 lt 7 lw lwidth pt 7 ps psize
set style line 4 lt 4 lw lwidth pt 13 ps psize
set style line 5 lt 9 lw lwidth pt 9 ps psize
set style line 6 lt 8 lw lwidth pt 8 ps psize
set style line 7 lt 10 lw lwidth pt 10 ps psize
# set key Left box
# set yrange [0:1.25]
# set ytics 0, 0.2, 1
set style increment user
set grid ytics lt 0 lw 1 lc rgb "#bbbbbb"
set grid xtics lt 0 lw 1 lc rgb "#bbbbbb"

simlog = "./phonesim_result.txt"

####################################
# plot total cost 
####################################
tcost_eps = "total_cost.eps"

set terminal postscript eps color enhanced "Times-Roman" fontsize
set output tcost_eps
#set xrange [0:11]
#set yrange [200:310]
set key right center box
set key width -5
set xlabel "Number of participating phones"
set ylabel "Total cost" 
plot simlog using 1:2 with linespoints t "MinSense Optimal", \
     simlog using 1:5 with linespoints t "FairSense Optimal", \
     simlog using 1:8 with linespoints t "Estimation-based Heuristic", \
     simlog using 1:23 with linespoints t "Adaptive cost Heuristic", \
     simlog using 1:41 with linespoints t "Baseline Heuristic"

####################################
# plot cost variance
####################################
costvar_eps = "cost_variance.eps"

set terminal postscript eps color enhanced "Times-Roman" fontsize
set output costvar_eps
#set xrange [0:11]
#set yrange [200:310]
set key at 74, 1.5
#set key right center box
set key width -5
set xlabel "Number of participating phones"
set ylabel "Phone cost variance" 
plot simlog using 1:4 with linespoints t "MinSense Optimal", \
     simlog using 1:7 with linespoints t "FairSense Optimal", \
     simlog using 1:10 with linespoints t "Estimation-based Heuristic", \
     simlog using 1:25 with linespoints t "Adaptive cost Heuristic", \
     simlog using 1:43 with linespoints t "Baseline Heuristic"

####################################
# plot max individual cost
####################################
maxindcost_eps = "max_ind_cost.eps"

set terminal postscript eps color enhanced "Times-Roman" fontsize
set output maxindcost_eps
#set xrange [0:11]
#set yrange [200:310]
#set key at 74, 1.5
set key default
set key right center box
set key width -5
set xlabel "Number of participating phones"
set ylabel "Max individual phone cost" 
plot simlog using 1:3 with linespoints t "MinSense Optimal", \
     simlog using 1:6 with linespoints t "FairSense Optimal", \
     simlog using 1:9 with linespoints t "Estimation-based Heuristic", \
     simlog using 1:24 with linespoints t "Adaptive cost Heuristic", \
     simlog using 1:42 with linespoints t "Baseline Heuristic"

###### Plots trend for adaptive cost heuristic ######
set xtics ("1.0" 0, "1.05" 1, "1.1" 2, "1.15" 3, "1.20" 4, "1.25" 5)
####################################
# plot variance trend
####################################
vartrend_log = "var_trend.txt"
vartrend_eps = "var_trend.eps"

set terminal postscript eps color enhanced "Times-Roman" fontsize
set output vartrend_eps
set yrange [0.14:0.47]
set key default
set key right top box
set xlabel "Cost multiplier"
set ylabel "Phone cost variance" 
plot vartrend_log using 1 with linespoints t "45 phones", \
     vartrend_log using 2 with linespoints t "50 phones", \
     vartrend_log using 3 with linespoints t "55 phones", \
     vartrend_log using 4 with linespoints t "60 phones", \
     vartrend_log using 5 with linespoints t "65 phones", \
     vartrend_log using 6 with linespoints t "70 phones", \
     vartrend_log using 7 with linespoints t "75 phones" 

####################################
# plot cost trend
####################################
costtrend_log = "cost_trend.txt"
costtrend_eps = "cost_trend.eps"

set terminal postscript eps color enhanced "Times-Roman" fontsize
set output costtrend_eps
set yrange [22:27.9]
set key default
set key right bottom box
set xlabel "Cost multiplier"
set ylabel "Phone cost variance" 
plot costtrend_log using 1 with linespoints t "45 phones", \
     costtrend_log using 2 with linespoints t "50 phones", \
     costtrend_log using 3 with linespoints t "55 phones", \
     costtrend_log using 4 with linespoints t "60 phones", \
     costtrend_log using 5 with linespoints t "65 phones", \
     costtrend_log using 6 with linespoints t "70 phones", \
     costtrend_log using 6 with linespoints t "75 phones" 
