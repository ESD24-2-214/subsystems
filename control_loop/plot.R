library(tidyverse)
my_data <- read_csv("angle_err_over_time.csv", col_names = TRUE)

dim(my_data)

plot(my_data)
