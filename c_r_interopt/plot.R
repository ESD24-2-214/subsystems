## library(tidyverse)

## # Read angle error data
## control_data <- read_csv("control_loop-0.csv", col_names = TRUE)

## # Clean column names (remove whitespace)
## names(control_data) <- trimws(names(control_data))

## # Plot 1: Angle Error Over Time
## p1 <- ggplot(data=control_data) +
##   ## geom_line(aes(x = Time, y = `Angle Error Norm`), color = "black", linewidth = 1) +
##   geom_point(aes(x = Time, y = `Angle Error Norm`), color = "black", size = 2) +
##   ## geom_line(aes(x = Time, y = `Angle Error e1e2`), color = "red", linewidth = 1) +
##   geom_point(aes(x = Time, y = `Angle Error e1e2`), color = "red", shape = 17, size = 2) +
##   labs(title = "Angle Errors Over Time",
##        x = "Time",
##        y = "Angle Error (radians)") +
##   theme_minimal() +
##   theme(
##     plot.title = element_text(size = 16, face = "bold", hjust = 0.5),
##     axis.title = element_text(size = 12, face = "bold"),
##     legend.position = "bottom"
##   )

## # Plot 2: Vector scatter plot with time gradient
## p2 <- ggplot() +
##   # Reference vectors (2D)
##   geom_segment(data = control_data,
##                aes(x = 0, y = 0, xend = `ref e1 world`, yend = `ref e2 world`),
##                arrow = arrow(length = unit(0.3, "cm")),
##                color = "gray50",
##                linewidth = 0.5) +
##   # Current vectors with time gradient (2D)
##   geom_segment(data = control_data,
##                aes(x = 0, y = 0, xend = `cur e1 world`, yend = `cur e2 world`, color = Time),
##                arrow = arrow(length = unit(0.3, "cm")),
##                linewidth = 0.7) +
##   scale_color_viridis_c(option = "plasma") +
##   coord_fixed(ratio = 1) +  # Ensure 1:1 aspect ratio
##   labs(title = "Vector Positions Over Time (2D Projection)",
##        x = "e1 Component",
##        y = "e2 Component",
##        color = "Time") +
##   theme_minimal() +
##   theme(
##     plot.title = element_text(size = 16, face = "bold", hjust = 0.5),
##     axis.title = element_text(size = 12, face = "bold"),
##     legend.position = "right"
##   ) +
##   # Add a unit circle for reference
##   geom_path(data = data.frame(
##     x = cos(seq(0, 2*pi, length.out = 100)),
##     y = sin(seq(0, 2*pi, length.out = 100))
##   ), aes(x = x, y = y), linetype = "dashed", color = "gray70")

## # Save plots to PDF
## ggsave("angle_errors_over_time.pdf", plot = p1, width = 10, height = 7)
## ggsave("vector_positions_over_time.pdf", plot = p2, width = 10, height = 8)

## # Display plots
## print(p1)
## print(p2)
library(tidyverse)
library(gridExtra) # For arranging multiple plots on one page
library(grid)
library(cowplot) # For plot_grid function

# Create an empty list to store all plots
angle_error_plots <- list()
vector_plots <- list()

# Loop through each control loop file (0-7)
for (i in 0:7) {
  # Read data
  filename <- paste0("control_loop-", i, ".csv")
  control_data <- read_csv(filename, col_names = TRUE)

  # Clean column names (remove whitespace)
  names(control_data) <- trimws(names(control_data))

  # Create angle error plot
  p1 <- ggplot(data=control_data) +
    geom_point(aes(x = Time, y = `Angle Error Norm`), color = "black", size = 1.5) +
    geom_point(aes(x = Time, y = `Angle Error e1e2`), color = "red", shape = 17, size = 1.5) +
    labs(title = paste("Control Loop", i, "- Angle Errors"),
         x = "Time",
         y = "Angle Error (radians)") +
    theme_minimal() +
    theme(
      plot.title = element_text(size = 12, face = "bold", hjust = 0.5),
      axis.title = element_text(size = 10),
      plot.margin = margin(5, 10, 5, 10)
    )

  # Create vector plot
  p2 <- ggplot() +
    # Reference vectors (2D)
    geom_segment(data = control_data,
                 aes(x = 0, y = 0, xend = `ref e1 world`, yend = `ref e2 world`),
                 arrow = arrow(length = unit(0.2, "cm")),
                 color = "gray50",
                 linewidth = 0.4) +
    # Current vectors with time gradient (2D)
    geom_segment(data = control_data,
                 aes(x = 0, y = 0, xend = `cur e1 world`, yend = `cur e2 world`, color = Time),
                 arrow = arrow(length = unit(0.2, "cm")),
                 linewidth = 0.5) +
    scale_color_viridis_c(option = "plasma") +
    coord_fixed(ratio = 1) +  # Ensure 1:1 aspect ratio
    labs(title = paste("Control Loop", i, "- Vector Positions"),
         x = "e1 Component",
         y = "e2 Component",
         color = "Time") +
    theme_minimal() +
    theme(
      plot.title = element_text(size = 12, face = "bold", hjust = 0.5),
      axis.title = element_text(size = 10),
      legend.position = "right",
      legend.key.size = unit(0.8, "lines"),
      legend.title = element_text(size = 8),
      legend.text = element_text(size = 7),
      plot.margin = margin(5, 10, 5, 10)
    ) +
    # Add a unit circle for reference
    geom_path(data = data.frame(
      x = cos(seq(0, 2*pi, length.out = 100)),
      y = sin(seq(0, 2*pi, length.out = 100))
    ), aes(x = x, y = y), linetype = "dashed", color = "gray70")

  # Store plots in lists
  angle_error_plots[[i+1]] <- p1
  vector_plots[[i+1]] <- p2
}

# Create combined plot pages
# First create a page with all angle error plots
angle_error_page <- plot_grid(
  plotlist = angle_error_plots,
  ncol = 2,
  nrow = 4
)

# Then create a page with all vector plots
vector_page <- plot_grid(
  plotlist = vector_plots,
  ncol = 2,
  nrow = 4
)

# Add title to each page
angle_error_page_with_title <- plot_grid(
  ggdraw() +
    draw_label("Angle Errors Across All Control Loops",
               fontface = "bold", size = 16, x = 0.5, y = 0.5),
  angle_error_page,
  ncol = 1,
  rel_heights = c(0.05, 0.95)
)

vector_page_with_title <- plot_grid(
  ggdraw() +
    draw_label("Vector Positions Across All Control Loops",
               fontface = "bold", size = 16, x = 0.5, y = 0.5),
  vector_page,
  ncol = 1,
  rel_heights = c(0.05, 0.95)
)

# Save as single multi-page PDF
pdf("control_loop_analysis.pdf", width = 12, height = 16)
print(angle_error_page_with_title)
print(vector_page_with_title)
dev.off()

# Display message
cat("PDF created with all plots as control_loop_analysis.pdf\n")
