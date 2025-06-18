#!/usr/bin/env python3

# --- Core Python/FastAPI Imports ---
import uvicorn
from fastapi import FastAPI, HTTPException
import threading
import numpy as np
import rclpy
from rclpy.node import Node

# --- ROS2 Imports ---
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
import tf_transformations

# =================================================================
# 1. GLOBAL ROS SETUP
#
# This setup is done once when the script starts.
# We initialize the ROS node and the TF listener.
# =================================================================

class TFAPIServer(Node):
    def __init__(self):
        super().__init__('tf_api_server')
        
        # Initialize the TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Initialize FastAPI app
        self.app = FastAPI(
            title="TF Transformation API",
            description="An API to get 4x4 transformation matrices from a running ROS2 TF tree.",
            version="1.0.0",
        )
        
        # Setup FastAPI routes
        self.setup_routes()
        
    def setup_routes(self):
        @self.app.get("/get_transform/{target_frame}/{source_frame}")
        async def get_transform(target_frame: str, source_frame: str):
            return await self.get_transformation_matrix(target_frame, source_frame)

# =================================================================
# 2. THE CORE TRANSFORMATION FUNCTION
#
# This function contains the actual logic for getting the transform.
# It's clean and can be reused elsewhere if needed.
# =================================================================

    async def get_transformation_matrix(self, target_frame, source_frame):
        """
        Looks up the transform between two frames and returns it as a 4x4 NumPy matrix.

        Args:
            target_frame (str): The name of the target coordinate frame.
            source_frame (str): The name of the source coordinate frame.
            
        Returns:
            dict: A dictionary containing the transformation matrix and metadata.
        """
        try:
            # Get the transform
            transform = await self.tf_buffer.lookup_transform_async(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )
            
            # Convert transform to 4x4 matrix
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            
            # Convert quaternion to rotation matrix
            q = [rotation.x, rotation.y, rotation.z, rotation.w]
            rot_matrix = tf_transformations.quaternion_matrix(q)
            
            # Create 4x4 transformation matrix
            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = rot_matrix[:3, :3]
            transform_matrix[0, 3] = translation.x
            transform_matrix[1, 3] = translation.y
            transform_matrix[2, 3] = translation.z
            
            return {
                "success": True,
                "transform": transform_matrix.tolist(),
                "source_frame": source_frame,
                "target_frame": target_frame,
                "timestamp": str(transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9)
            }
            
        except TransformException as e:
            raise HTTPException(status_code=404, detail=f"Transform failed: {str(e)}")
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        # Create and initialize the TF API server
        tf_api_server = TFAPIServer()
        
        # Start the FastAPI server in a separate thread
        config = uvicorn.Config(
            app=tf_api_server.app,
            host="0.0.0.0",
            port=8000,
            log_level="info"
        )
        server = uvicorn.Server(config)
        
        # Run the server in a separate thread
        import threading
        server_thread = threading.Thread(target=server.run)
        server_thread.daemon = True
        server_thread.start()
        
        # Spin the ROS2 node
        rclpy.spin(tf_api_server)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        # Cleanup
        if 'tf_api_server' in locals():
            tf_api_server.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()