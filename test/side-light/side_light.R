library(tidyverse)

# Get a list of all CSV files
csv_files <- list.files(pattern = "sun_sensor_side_light_\\d+\\.csv")

# Function to read a single CSV file and calculate means
process_file <- function(file) {
  # Read the CSV file
  data <- read_csv(file, col_types = cols(
    Iteration = col_double(),
    Time = col_double(),
    e1 = col_double(),
    e2 = col_double(),
    e3 = col_double()
  ))

  # Calculate means for e1, e2, e3
  means <- data %>%
    summarise(
      file = file,
      mean_e1 = mean(e1, na.rm = TRUE),
      mean_e2 = mean(e2, na.rm = TRUE),
      mean_e3 = mean(e3, na.rm = TRUE)
    )

  return(means)
}

# Apply the function to all files and combine results
all_means <- map_df(csv_files, process_file)

# Display means for each file
print("Means for each file:")
print(all_means)

# Calculate the mean of means
mean_of_means <- all_means %>%
  summarise(
    overall_mean_e1 = mean(mean_e1, na.rm = TRUE),
    overall_mean_e2 = mean(mean_e2, na.rm = TRUE),
    overall_mean_e3 = mean(mean_e3, na.rm = TRUE)
  )

# Display the mean of means
print("Mean of means:")
print(mean_of_means)

# For visualization, we can also create a longer format dataframe
all_means_long <- all_means %>%
  pivot_longer(
    cols = starts_with("mean_"),
    names_to = "sensor",
    values_to = "mean_value"
  ) %>%
  mutate(sensor = str_replace(sensor, "mean_", ""))

# Plot means by sensor and file
ggplot(all_means_long, aes(x = file, y = mean_value, fill = sensor)) +
  geom_bar(stat = "identity", position = "dodge") +
  theme_minimal() +
  labs(
    title = "Mean Sensor Values by File",
    x = "File",
    y = "Mean Value",
    fill = "Sensor"
  ) +
  theme(axis.text.x = element_text(angle = 45, hjust = 1))
