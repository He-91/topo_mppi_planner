# Single dynamic obstacle state with prediction
int32 id
geometry_msgs/Point position          # Current position
geometry_msgs/Vector3 velocity        # Estimated velocity
float64 radius                        # Obstacle radius
float64 height                        # Obstacle height
geometry_msgs/Point[] predicted_positions  # Predicted future positions (2s horizon, 0.1s dt = 20 steps)
