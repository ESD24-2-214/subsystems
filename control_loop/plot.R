library(tidyverse)
my_data <- read_csv("angle_err_over_time.csv", col_names = TRUE)
print(my_data)

names(my_data) <- trimws(names(my_data))

p1 <- ggplot(my_data) +
  geom_point(aes(x = Time, y = `Angle Error Norm`), color = "black", size = 2) +
  geom_point(aes(x = Time, y = `Angle Error e1e2`), color = "red", shape = 17,
             size = 1, show.legend = TRUE, ) +
  ## geom_point(aes(x = Time, y = `Angle Error e3e1`), color = "blue",
  ##            shape = 10) +
  ## geom_point(aes(x = Time, y = `Angle Error e2e3`), color = "green",
  ##            shape = 12) +
  labs(title = "Angle Errors Over Time",
       x = "Time",
       y = "Angle Error") +
  theme_minimal()
  ## theme(
  ##   legend.position = "right",
  ##   legend.background = element_rect(fill = "white",
  ##                                    color = "black",
  ##                                    linewidth = 0.5),
  ##   legend.title = element_text(face = "bold"),
  ##   legend.key = element_rect(fill = "white"),
  ##   legend.margin = margin(6, 6, 6, 6)
  ## ) +
  ## # Optional: Use a nicer color palette
  ## scale_color_brewer(palette = "Set1")

print(p1)
