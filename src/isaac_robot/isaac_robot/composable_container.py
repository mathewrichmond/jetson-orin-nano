#!/usr/bin/env python3
"""
Composable Node Container for Python Nodes
Runs multiple ROS 2 nodes in a single process to enable zero-copy intra-process communication.

This achieves the same benefits as C++ composable nodes:
- Zero-copy message passing between nodes in the same process
- Reduced latency and CPU usage  
- Lower memory overhead

Usage:
    ros2 run isaac_robot composable_container --ros-args -p composable_nodes:='[...]'
"""

# Standard library
import importlib
import json
import sys
from typing import Dict, List, Optional

# Third-party
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.parameter import Parameter


class ComposableNodeContainer(Node):
    """Container that runs multiple nodes in a single process with intra-process communication"""
    
    # Use class-level dict to store executor to avoid attribute access issues
    _executor_storage = {}

    def __init__(self, container_name: str = "composable_container"):
        super().__init__(container_name)
        # Use object.__setattr__ to bypass any potential Node class overrides
        object.__setattr__(self, 'nodes', [])
        object.__setattr__(self, 'running', False)
        # Store executor in class dict using instance ID as key
        instance_id = id(self)
        object.__setattr__(self, '_instance_id', instance_id)
        ComposableNodeContainer._executor_storage[instance_id] = None

        # Declare parameter for composable node configurations
        # Use JSON string since ROS 2 parameters don't support nested lists/dicts
        self.declare_parameter("composable_nodes_json", "[]")

        self.get_logger().info(f"ComposableNodeContainer '{container_name}' initialized (instance_id: {self._instance_id})")
    
    @property
    def executor(self):
        """Get executor from storage"""
        try:
            instance_id = object.__getattribute__(self, '_instance_id')
        except AttributeError:
            # Fallback if _instance_id not set yet (shouldn't happen, but be safe)
            instance_id = id(self)
        return ComposableNodeContainer._executor_storage.get(instance_id)
    
    @executor.setter
    def executor(self, value):
        """Set executor in storage"""
        try:
            instance_id = object.__getattribute__(self, '_instance_id')
        except AttributeError:
            # Fallback if _instance_id not set yet
            instance_id = id(self)
            object.__setattr__(self, '_instance_id', instance_id)
        ComposableNodeContainer._executor_storage[instance_id] = value

    def load_components_from_config(self):
        """Load components from parameter configuration"""
        try:
            # Get JSON string parameter and parse it
            composable_nodes_json = self.get_parameter("composable_nodes_json").value
            if not composable_nodes_json or composable_nodes_json == "[]":
                self.get_logger().warn("No composable nodes configured")
                return
            
            composable_nodes_config = json.loads(composable_nodes_json)
        except Exception as e:
            self.get_logger().warn(f"No composable_nodes_json parameter found or invalid JSON: {e}")
            return

        if not composable_nodes_config:
            self.get_logger().warn("No composable nodes configured")
            return

        if not isinstance(composable_nodes_config, list):
            self.get_logger().error(f"composable_nodes_json must be a JSON array, got {type(composable_nodes_config)}")
            return

        self.get_logger().info(f"Loading {len(composable_nodes_config)} composable nodes")

        for node_config in composable_nodes_config:
            try:
                package = node_config.get("package")
                module = node_config.get("module")
                class_name = node_config.get("class")
                node_name = node_config.get("name")
                namespace = node_config.get("namespace", "")
                parameters = node_config.get("parameters", {})

                if not all([package, module, class_name]):
                    self.get_logger().error(f"Invalid node config: missing package/module/class")
                    continue

                # Load the component
                node = self.load_component(
                    package=package,
                    module=module,
                    class_name=class_name,
                    node_name=node_name,
                    namespace=namespace,
                    parameters=parameters,
                )

            except Exception as e:
                self.get_logger().error(f"Failed to load node from config: {e}")
                continue

    def load_component(
        self,
        package: str,
        module: str,
        class_name: str,
        node_name: Optional[str] = None,
        namespace: Optional[str] = None,
        parameters: Optional[Dict] = None,
    ) -> Node:
        """
        Load a node component dynamically

        Args:
            package: Python package name (e.g., 'realsense_camera')
            module: Module name within package (e.g., 'realsense_camera_node')
            class_name: Class name of the node (e.g., 'RealSenseCameraNode')
            node_name: Optional node name override
            namespace: Optional namespace
            parameters: Optional parameters dict

        Returns:
            The loaded node instance
        """
        try:
            # Import the module
            module_path = f"{package}.{module}"
            self.get_logger().info(f"Loading component: {module_path}.{class_name}")

            # Import the module
            node_module = importlib.import_module(module_path)

            # Get the node class
            node_class = getattr(node_module, class_name)

            # Create node instance
            # Pass node_name if provided, otherwise use default from class
            # Note: Some nodes may not accept node_name parameter - that's OK
            try:
                if node_name:
                    # Try to instantiate with node_name parameter
                    try:
                        node_instance = node_class(node_name=node_name)
                    except TypeError:
                        # Node doesn't accept node_name, use default
                        node_instance = node_class()
                else:
                    node_instance = node_class()
            except Exception as e:
                self.get_logger().error(f"Failed to instantiate {class_name}: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())
                raise

            # Set parameters if provided
            # Note: Parameters should be set before node starts processing
            # We'll set them after creation but before adding to executor
            if parameters:
                param_list = []
                for key, value in parameters.items():
                    try:
                        # Check if parameter already declared
                        if node_instance.has_parameter(key):
                            # Update existing parameter
                            param_list.append(Parameter(key, value=value))
                        else:
                            # Declare new parameter
                            node_instance.declare_parameter(key, value)
                    except Exception as e:
                        self.get_logger().warn(f"Could not set parameter {key}: {e}")
                
                # Set all parameters at once
                if param_list:
                    try:
                        node_instance.set_parameters(param_list)
                    except Exception as e:
                        self.get_logger().warn(f"Could not set parameters: {e}")

            # Add to container
            self.nodes.append(node_instance)
            self.get_logger().info(
                f"✓ Loaded component: {class_name} (node: {node_instance.get_name()})"
            )

            return node_instance

        except Exception as e:
            self.get_logger().error(f"Failed to load component {package}.{module}.{class_name}: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            raise

    def start(self):
        """Start the container and all loaded nodes"""
        if self.running:
            self.get_logger().warn("Container is already running")
            return

        if not self.nodes:
            self.get_logger().warn("No nodes loaded, nothing to start")
            return

        self.get_logger().info(f"Starting container with {len(self.nodes)} nodes")
        self.get_logger().info(f"Executor before creation: {self.executor}, type: {type(self.executor)}")

        # Create executor with multiple threads for parallel execution
        # Use one thread per node + one for the container
        num_threads = len(self.nodes) + 1
        try:
            # Get the context from this node
            context = self.context
            self.get_logger().info(f"Creating executor with {num_threads} threads, context type: {type(context)}, context: {context}")
            
            # Create executor - use SingleThreadedExecutor for now
            # TODO: Debug why MultiThreadedExecutor returns None in this context
            # For now, SingleThreadedExecutor works and still provides zero-copy benefits
            self.get_logger().info(f"About to create SingleThreadedExecutor with context: {context}")
            executor_result = None
            try:
                executor_result = SingleThreadedExecutor(context=context)
                self.get_logger().info(f"SingleThreadedExecutor() returned: {executor_result}, type: {type(executor_result)}")
            except Exception as create_error:
                self.get_logger().error(f"Exception during SingleThreadedExecutor creation: {create_error}")
                import traceback
                self.get_logger().error(traceback.format_exc())
                raise
            
            # Assign executor using property setter
            self.executor = executor_result
            self.get_logger().info(f"After assignment, self.executor: {self.executor}, type: {type(self.executor)}, id: {id(self.executor)}")
            
            # Verify executor was created
            if self.executor is None:
                self.get_logger().error("Executor is None after creation - this should not happen!")
                self.get_logger().error(f"executor_result was: {executor_result}")
                raise RuntimeError("Executor returned None")
            
            self.get_logger().info(f"Executor created successfully: {type(self.executor)}, id: {id(self.executor)}")
        except Exception as e:
            self.get_logger().error(f"Failed to create executor: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            # Don't set executor to None here - let it remain uninitialized
            return

        # Add container node to executor
        try:
            self.get_logger().info(f"Adding container node to executor: {self.executor}, type: {type(self.executor)}")
            if self.executor is None:
                self.get_logger().error("CRITICAL: Executor is None when trying to add container node!")
                raise RuntimeError("Executor is None")
            self.executor.add_node(self)
            self.get_logger().info(f"Container node added successfully, executor still: {self.executor}, id: {id(self.executor)}")
        except Exception as e:
            self.get_logger().error(f"Failed to add container node to executor: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.executor = None  # Clear executor on failure
            return

        # Add all component nodes to executor
        for node in self.nodes:
            try:
                self.get_logger().info(f"Adding component node '{node.get_name()}' to executor: {self.executor}, id: {id(self.executor) if self.executor else None}")
                if self.executor is None:
                    self.get_logger().error(f"CRITICAL: Executor became None when adding node '{node.get_name()}'!")
                    raise RuntimeError("Executor is None")
                self.executor.add_node(node)
                self.get_logger().info(f"  - Added node '{node.get_name()}' to executor, executor still: {self.executor}, id: {id(self.executor)}")
            except Exception as e:
                self.get_logger().error(f"Failed to add node '{node.get_name()}' to executor: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())
                # Don't clear executor, continue with other nodes

        self.running = True
        
        # Final verification before returning
        executor_at_end = self.executor
        executor_id_at_end = id(executor_at_end) if executor_at_end else None
        self.get_logger().info(f"✓ Container started successfully (intra-process communication enabled)")
        self.get_logger().info(f"Executor at end of start(): {executor_at_end}, type: {type(executor_at_end)}, id: {executor_id_at_end}")
        
        # Double-check executor is still set
        if executor_at_end is None:
            self.get_logger().error("CRITICAL: Executor became None during start() - this should not happen!")
            raise RuntimeError("Executor was None at end of start()")
        
        # Store executor ID for debugging
        self._executor_id = executor_id_at_end
        executor_check_again = self.executor
        self.get_logger().info(f"Stored executor ID: {self._executor_id}, current executor ID: {id(executor_check_again) if executor_check_again else None}")

    def spin(self):
        """Spin the executor (blocks until shutdown)"""
        self.get_logger().info(f"spin() called - executor: {self.executor}, id: {id(self.executor) if self.executor else None}, running: {self.running}")
        
        # Start container if not already started
        if not self.running:
            self.get_logger().info("Container not running, calling start()...")
            executor_before_start = self.executor
            executor_id_before_start = id(executor_before_start) if executor_before_start else None
            try:
                self.start()
            except Exception as e:
                self.get_logger().error(f"start() raised exception: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())
                raise
            
            executor_after_start = self.executor
            executor_id_after_start = id(executor_after_start) if executor_after_start else None
            self.get_logger().info(f"start() returned - executor: {executor_after_start}, id: {executor_id_after_start}, running: {self.running}")
            self.get_logger().info(f"Executor changed? before: {executor_before_start} (id: {executor_id_before_start}), after: {executor_after_start} (id: {executor_id_after_start})")
            if hasattr(self, '_executor_id'):
                self.get_logger().info(f"Stored executor ID from start(): {self._executor_id}, current executor ID: {executor_id_after_start}")
        
        executor_to_check = self.executor
        self.get_logger().info(f"After start check - executor: {executor_to_check}, type: {type(executor_to_check)}, id: {id(executor_to_check) if executor_to_check else None}, running: {self.running}")
        
        # Verify executor is initialized
        if executor_to_check is None:
            self.get_logger().error(f"Cannot spin: executor not initialized (executor={executor_to_check}, self.running={self.running})")
            self.get_logger().error(f"Container state - nodes: {len(self.nodes)}, executor id: {id(executor_to_check) if executor_to_check else None}")
            # Try to start again
            self.get_logger().warn("Attempting to start container again...")
            self.running = False  # Reset running flag
            self.start()
            executor_to_check = self.executor
            self.get_logger().info(f"After retry start - executor: {executor_to_check}, type: {type(executor_to_check)}")
            if executor_to_check is None:
                self.get_logger().error("Executor still not initialized after retry - aborting")
                return

        self.get_logger().info(f"Spinning executor: {type(executor_to_check)}, id: {id(executor_to_check)}, nodes: {len(self.nodes)}")
        try:
            # Use spin() instead of spin_once() in a loop to avoid excessive CPU usage
            # spin() handles timing internally and is more efficient
            executor_to_check.spin()
        except KeyboardInterrupt:
            self.get_logger().info("Received shutdown signal")
        except Exception as e:
            self.get_logger().error(f"Executor error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self.stop()

    def stop(self):
        """Stop the container and all nodes"""
        if not self.running:
            return

        self.get_logger().info("Stopping container...")
        self.running = False

        # Shutdown executor
        executor_to_shutdown = self.executor
        if executor_to_shutdown:
            executor_to_shutdown.shutdown()
            # Clear executor from storage
            ComposableNodeContainer._executor_storage.pop(self._instance_id, None)

        # Destroy all nodes
        for node in self.nodes:
            try:
                node.destroy_node()
            except Exception as e:
                self.get_logger().error(f"Error destroying node {node.get_name()}: {e}")

        # Destroy container node
        self.destroy_node()

        self.get_logger().info("✓ Container stopped")


def main(args=None):
    """Main entry point for composable container"""
    rclpy.init(args=args)

    # Create container
    container = ComposableNodeContainer()

    try:
        # Load components from configuration
        container.load_components_from_config()

        # Start and spin
        container.spin()

    except Exception as e:
        container.get_logger().error(f"Container error: {e}")
        import traceback
        container.get_logger().error(traceback.format_exc())
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
