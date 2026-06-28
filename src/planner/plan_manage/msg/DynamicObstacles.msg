# Array of dynamic obstacle states for MPPI planner consumption
Header header
float64 prediction_horizon            # Prediction horizon in seconds (e.g., 2.0)
float64 prediction_dt                 # Prediction time step in seconds (e.g., 0.1)
int32 prediction_steps                # Number of prediction steps (e.g., 20)
ddo_planner/DynamicObstacle[] obstacles
