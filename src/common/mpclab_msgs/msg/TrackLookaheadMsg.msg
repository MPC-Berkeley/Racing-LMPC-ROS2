# This is a message to hold data with track lookahead
std_msgs/Header header
float64 t         # Time
float64 l         # Length of lookahead in m
float64 dl        # discretization step-size in meters
float64 n         # number of looakhead entries
float64[] curvature      # Curvature Lookahead
